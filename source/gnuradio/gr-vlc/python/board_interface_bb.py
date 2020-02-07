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
from threading import Thread
from threading import Semaphore
import pmt
from enum import Enum
import socket


class board_interface_bb(gr.basic_block):
    """
    Interfaces with a vlc board. Receives commands on the message port or data on the stream port. The input data
    must be a stream of bytes representing the wave with tags for start and end of the packet. Send backs messages
    when the current operation is done. In receiver mode outputs a stream of bytes representing the sampled signal.
    """

    class TransmitterStatus(Enum):
        IDLE = 0,
        LOOK_FOR_END = 1,
        PREPARE = 2,
        SENDING = 3

    class ReceiverStatus(Enum):
        IDLE = 0,
        RECEIVING = 1

    class InterfaceMode(Enum):
        IDLE = 0,
        TRANSMITTER = 1,
        RECEIVER = 2

    def __init__(self, address: str, cmd_port: int, data_port: int, block_size: int):
        gr.basic_block.__init__(self,
                                name="board_interface_bb",
                                in_sig=[numpy.uint8],
                                out_sig=[numpy.uint8])

        self.board_address = address
        self.cmd_port = cmd_port
        self.data_port = data_port
        self.block_size = block_size

        self.port_in_name = 'msg_in'
        self.port_out_name = 'msg_out'

        self.message_port_register_in(pmt.intern(self.port_in_name))
        self.message_port_register_out(pmt.intern(self.port_out_name))
        self.set_msg_handler(pmt.intern(self.port_in_name), self.handle_msg)

        self.ready_packets = Semaphore(0)
        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_addr = (self.board_address, self.cmd_port)
        self.data_addr = (self.board_address, self.data_port)

        self.mode = self.InterfaceMode.IDLE

        self.rx_thread = None
        self.tx_thread = None

        # Receiver part
        self.receiver_status = self.ReceiverStatus.IDLE
        self.received_data: numpy.ndarray = numpy.zeros(0, dtype=numpy.uint8)

        # Transmitter part
        self.transmitter_status = self.TransmitterStatus.IDLE
        self.start_tag = None
        self.end_tag = None
        self.current_packet: numpy.ndarray = None

        self.start_loops()

    def set_mode_idle(self):
        self.mode = self.InterfaceMode.IDLE

        print('Sending set idle command to board!')
        buffer_command = numpy.array([0, 0], dtype=numpy.uint32)
        self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)
        _reply = self.sock_cmd.recv(8)

    def set_mode_tx(self):
        self.mode = self.InterfaceMode.TRANSMITTER

        print('Sending set tx command to board!')
        buffer_command = numpy.array([0, 1], dtype=numpy.uint32)
        self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)
        _reply = self.sock_cmd.recv(8)

    def set_mode_rx(self):
        self.mode = self.InterfaceMode.RECEIVER

        print('Sending set rx command to board!')
        buffer_command = numpy.array([0, 2], dtype=numpy.uint32)
        self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)

        print('Begin data reception!')
        buffer_command = numpy.array([1], dtype=numpy.uint32)
        self.sock_data.sendto(buffer_command.tobytes(), self.data_addr)

        self.receiver_status = self.ReceiverStatus.RECEIVING

    def transmit_last_wave(self):
        print('Retransmitting last wave')

        buffer_command = numpy.array([4, 0], dtype=numpy.uint32)
        self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)

        self.transmitter_status = self.TransmitterStatus.SENDING

        reply = numpy.frombuffer(self.sock_cmd.recv(8), dtype=numpy.uint32)
        if reply[1] == 0:
            _reply = self.sock_cmd.recv(8)

        self.transmitter_status = self.TransmitterStatus.IDLE

    def execute_cmd(self, command):
        cmd = command[0]
        arg = command[1]

        self.mode = self.InterfaceMode.IDLE
        self.receiver_status = self.ReceiverStatus.IDLE
        self.transmitter_status = self.TransmitterStatus.IDLE

        self.received_data = numpy.zeros(0, dtype=numpy.uint8)

        if cmd == 0:
            if arg == 0:
                self.set_mode_idle()
            elif arg == 1:
                self.set_mode_tx()
            elif arg == 2:
                self.set_mode_rx()
        elif cmd == 4:
            self.transmit_last_wave()

    def handle_rf_pkt(self, pkt):
        rf_cmd = pkt[0]

        if rf_cmd == 2:
            payload_len = len(pkt[1:])

            print(f'Sending packet {pkt} of len {payload_len}')

            buffer_command = numpy.array([2, payload_len], dtype=numpy.uint32)
            self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)
            _reply = self.sock_cmd.recv(8)

            self.sock_data.sendto(pkt[1:], self.data_addr)
            _reply = self.sock_cmd.recv(8)

            self.message_port_pub(pmt.intern(self.port_out_name), pmt.intern('done'))
        elif rf_cmd == 3:
            buffer_command = numpy.array([3, 0], dtype=numpy.uint32)
            self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)
            reply = numpy.frombuffer(self.sock_cmd.recv(8), dtype=numpy.uint32)

            pkt_len = reply[1]
            # TODO message must be more meaningful
            if pkt_len == 0:
                self.message_port_pub(pmt.intern(self.port_out_name), pmt.intern('done'))
            elif pkt_len > 0:
                rf_pkt = self.sock_data.recv(pkt_len)
                self.message_port_pub(pmt.intern(self.port_out_name), pmt.intern('done'))

    def handle_msg(self, msg_pmt):
        print(msg_pmt)

        car = pmt.to_python(pmt.car(msg_pmt))
        if car == 'cmd':
            self.execute_cmd(bytes.fromhex(pmt.to_python(pmt.cdr(msg_pmt))))
            self.message_port_pub(pmt.intern(self.port_out_name), pmt.intern('done'))

        elif car == 'rf':
            self.handle_rf_pkt(bytes.fromhex(pmt.to_python(pmt.cdr(msg_pmt))))

    def start_loops(self):
        self.rx_thread = Thread(target=self.start_receiver_loop, daemon=True)
        self.rx_thread.start()

        self.tx_thread = Thread(target=self.start_tx_waiter, daemon=True)
        self.tx_thread.start()

    def start_receiver_loop(self):
        while True:
            if self.receiver_status == self.ReceiverStatus.RECEIVING:
                data = self.sock_data.recv(self.block_size)
                self.received_data = numpy.append(self.received_data, numpy.frombuffer(data, dtype=numpy.uint8))

    def start_tx_waiter(self):

        while True:
            self.ready_packets.acquire()
            print(f'Packet received')

            print(self.current_packet.size)
            buffer_command = numpy.array([1, self.current_packet.size], dtype=numpy.uint32)
            self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)
            _reply = self.sock_cmd.recv(8)

            sent_data = 0

            while sent_data < self.current_packet.size:
                # It's ok to send more than the size of the array since it gets clipped anyway
                self.sock_data.sendto(self.current_packet[sent_data:sent_data + self.block_size].tobytes(),
                                      self.data_addr)
                sent_data += self.block_size

                _reply = self.sock_cmd.recv(8)

            self.message_port_pub(pmt.intern(self.port_out_name), pmt.cons(pmt.intern('ctrl'), pmt.intern('done')))
            self.transmitter_status = self.TransmitterStatus.IDLE

    def transmitter_work(self, input_items, output_items):
        in0 = input_items[0]
        in0_items_no = len(in0)

        first_item_offset = self.nitems_read(0)
        consumed_items = 0

        # Start by looking for the start tag of the packet
        if self.transmitter_status == self.TransmitterStatus.IDLE:
            tags = self.get_tags_in_window(0, 0, in0_items_no,
                                           pmt.intern(f'Packet start'))
            if tags:
                self.start_tag = gr.tag_to_python(tags[0])
                print(f'Tag key: {self.start_tag.key} tag Value: {self.start_tag.value}\
                            tag offset: {self.start_tag.offset}')

                self.current_packet = numpy.frombuffer(in0[self.start_tag.offset - first_item_offset:],
                                                       dtype=numpy.uint8)
                self.transmitter_status = self.TransmitterStatus.LOOK_FOR_END

                consumed_items = self.current_packet.size + self.start_tag.offset - first_item_offset

        # If we found the start of the packet, look for the ending tag
        if self.transmitter_status == self.TransmitterStatus.LOOK_FOR_END:
            tags = self.get_tags_in_window(0, 0, in0_items_no,
                                           pmt.intern(f'Packet end'))

            # Append all the data that we received to the current packet
            self.current_packet = numpy.append(self.current_packet,
                                               numpy.frombuffer(in0[consumed_items:], dtype=numpy.uint8))

            if tags:
                # If we find the ending tag then we can trim the array to the packet size and prepare for the next phase
                self.end_tag = gr.tag_to_python(tags[0])
                print(f'Tag key: {self.end_tag.key} tag Value: {self.end_tag.value}\
                            tag offset: {self.end_tag.offset}')

                self.transmitter_status = self.TransmitterStatus.PREPARE
                self.current_packet = self.current_packet[:(self.end_tag.offset - self.start_tag.offset)]
                print(f'Releasing semaphore')
                self.ready_packets.release()

        # print(f'Board interface: consumed {in0_items_no} items')
        self.consume(0, in0_items_no)
        return 0

    def receiver_work(self, input_items, output_items):
        # print(f'Board interface: rx work')
        out = output_items[0]
        out_len = len(out)

        current_items_len = self.received_data.size
        written_items = min(out_len, current_items_len)

        out[:written_items] = self.received_data[:written_items]
        self.received_data = self.received_data[written_items:]

        self.consume(0, 0)
        return written_items

    def forecast(self, noutput_items, ninput_items_required):
        noutput_items = self.received_data.size
        ninput_items_required[0] = 0

    def general_work(self, input_items, output_items):
        # print(f'Board interface: general work')

        if self.mode == self.InterfaceMode.RECEIVER:
            return self.receiver_work(input_items, output_items)
        elif self.mode == self.InterfaceMode.TRANSMITTER:
            return self.transmitter_work(input_items, output_items)

        return 0

    def work(self, input_items, output_items):
        # print(f'Board interface: general work')

        if self.mode == self.InterfaceMode.RECEIVER:
            return self.receiver_work(input_items, output_items)
        elif self.mode == self.InterfaceMode.TRANSMITTER:
            return self.transmitter_work(input_items, output_items)

        return 0
