#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2019 JoJo.
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#


import numpy
from gnuradio import gr
import pmt
from threading import Thread, Timer
import asyncio
import os
from typing import List
from gnuradio.gr import packet_utils
from enum import Enum
import json
from threading import Semaphore


class nic_interface_b(gr.sync_block):
    """
    Class to interface with the vlc Data Link Layer protocol. Exchanges data with a VLCNIC that are either packets
    or commands.
    """

    class Status(Enum):
        IDLE = 0,
        SENDING = 1,
        RECEIVING = 2,
        WAITING = 3

    def __init__(self, address: str = 'localhost', cmd_port: int = 0, data_port: int = 0):
        gr.sync_block.__init__(self,
                               name="nic_interface_b",
                               in_sig=[],
                               out_sig=[numpy.uint8])

        # Transmitter part
        self.preamble = b'\xff\xff\x55\x55\x00\x00'
        self.max_payload_size = 256
        self.overhead_size = 18

        self.out_packet: numpy.ndarray = None
        self.received_packets = 0

        # Receiver Part
        self.in_packets: asyncio.Queue[numpy.ndarray] = asyncio.Queue()
        self.pkt_port_in_name = 'pkt_in'

        self.message_port_register_in(pmt.intern(self.pkt_port_in_name))
        self.set_msg_handler(pmt.intern(self.pkt_port_in_name), self.handle_pkt)

        # Common Part
        self.status = self.Status.IDLE
        self.server = None
        self.address = address
        self.cmd_port = cmd_port
        self.data_port = data_port
        self.cmd_done_sem = Semaphore(0)

        self.port_in_name = 'msg_in'
        self.port_out_name = 'msg_out'
        self.message_port_register_in(pmt.intern(self.port_in_name))
        self.message_port_register_out(pmt.intern(self.port_out_name))

        self.set_msg_handler(pmt.intern(self.port_in_name), self.handle_msg)

        t = Thread(target=self.start_background_loop, daemon=True)
        t.start()

    def handle_pkt(self, pkt_pmt):
        """
        Handles packet received on the packet port

        :param pkt_pmt:
        """
        print(pkt_pmt)

        car = pmt.to_python(pmt.car(pkt_pmt))
        if car == 'pkt':
            data = pmt.to_python(pmt.cdr(pkt_pmt))
            self.in_packets.put_nowait(numpy.frombuffer(data, dtype=numpy.uint8))

    def handle_msg(self, msg_pmt):
        """
        Handles messages received on the general message port

        :param msg_pmt:
        """
        print(msg_pmt)

        car = pmt.to_python(pmt.car(msg_pmt))
        if car == 'ctrl':
            ctrl = pmt.to_python(pmt.cdr(msg_pmt))
            if ctrl == 'done':
                self.status = self.Status.IDLE
                self.cmd_done_sem.release()

    async def start_handlers(self):
        await asyncio.gather(self.open_cmd_socket(), self.open_pkt_socket())

    def start_background_loop(self) -> None:
        asyncio.run(self.open_cmd_socket())

    async def process_cmd(self, msg):
        print(f'Received cmd: {msg}')
        cmd = msg[0]

        # Data packet
        if cmd == 0:
            print(f'Sending packet to modulator')
            # TODO: eeeeh not really
            data = self.preamble + msg[1:]

            # print(f"Received {data.hex()}")
            data = numpy.frombuffer(data, dtype=numpy.uint8)

            self.out_packet = data

            self.status = self.Status.SENDING

        # Pass to board interface
        elif cmd == 1:
            print(f'Passing message to board interface')
            message = pmt.cons(pmt.intern('cmd'), pmt.to_pmt(msg[1:].hex()))
            self.message_port_pub(pmt.intern(self.port_out_name), message)

        # Pass rf pkt to board interface
        elif cmd == 2:
            print(f'Passing rf pkt to board interface')
            message = pmt.cons(pmt.intern('rf'), pmt.to_pmt(msg[1:].hex()))
            self.message_port_pub(pmt.intern(self.port_out_name), message)

    async def handle_vlc_cmd_client(self, reader, writer):
        while True:
            try:
                msg = await reader.read(2048)
                if len(msg) > 0:
                    try:
                        await self.process_cmd(msg)
                        self.cmd_done_sem.acquire()
                    except Exception as e:
                        print(e)
                else:
                    return
            except:
                return

            writer.write(b'done')
            await writer.drain()

    async def open_cmd_socket(self):
        """
        Start the server exchanging control message
        """
        self.server = await asyncio.start_server(
            self.handle_vlc_cmd_client, self.address, port=self.cmd_port)

        addr = self.server.sockets[0].getsockname()
        print(f'Serving on {addr}')

        async with self.server:
            await self.server.serve_forever()

    async def handle_vlc_pkt_client(self, reader, writer):
        while True:
            try:
                msg = await self.in_packets.get()
                writer.write(msg.tobytes())
                await writer.drain()
            except:
                return

    async def open_pkt_socket(self):
        """
        Start the server sending packets received from the demodulator
        """
        self.server = await asyncio.start_server(
            self.handle_vlc_pkt_client, self.address, port=self.data_port)

        addr = self.server.sockets[0].getsockname()
        print(f'Serving on {addr}')

        async with self.server:
            await self.server.serve_forever()

    def work(self, input_items, output_items):
        """
        Send out packets to be encoded and sent to the transmitter

        :param input_items:
        :param output_items:
        :return:
        """
        out = output_items[0]

        if not self.out_packet:
            return 0
        else:
            packet_len = self.out_packet.size
            out[:packet_len] = self.out_packet
            out[packet_len + 1] = 0

            self.add_item_tag(0,  # Port number
                              self.nitems_written(0),  # Offset
                              pmt.intern(f'Packet start'),
                              pmt.to_pmt(self.received_packets))

            self.add_item_tag(0,  # Port number
                              self.nitems_written(0) + packet_len,  # Offset
                              pmt.intern(f'Packet end'),
                              pmt.to_pmt(self.received_packets))

            self.received_packets += 1
            self.out_packet = None
            self.status = self.Status.WAITING

            print(f'Pkt source: output of {packet_len} items')
            return packet_len + 1
