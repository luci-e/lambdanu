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
from enum import Enum


class rx_bs2pkts_b(gr.sync_decimator):
    """
    A utility block to avoid loops, receives a bytestream and outputs the packets on the message port.
    """

    class StreamStatus(Enum):
        IDLE = 0,
        LOOK_FOR_END = 1,

    def __init__(self):
        gr.sync_decimator.__init__(self,
                                   name="rx_bs2pkts_b",
                                   in_sig=[numpy.uint8],
                                   out_sig=[])

        self.port_out_name = 'pkts_out'
        self.message_port_register_out(pmt.intern(self.port_out_name))

        self.status = self.StreamStatus.IDLE
        self.start_tag = None
        self.end_tag = None
        self.current_packet: numpy.array = None

    def send_packet_msg(self):
        print(f'Passing packet to nic interface')
        message = pmt.cons(pmt.intern('pkt'), pmt.to_pmt(self.current_packet.tobytes()))

    def work(self, input_items, output_items):
        in0 = input_items[0]
        in0_items_no = len(in0)

        first_item_offset = self.nitems_read(0)
        consumed_items = 0

        # Start by looking for the start tag of the packet
        if self.status == self.StreamStatus.IDLE:
            tags = self.get_tags_in_window(0, 0, in0_items_no,
                                           pmt.intern(f'Packet start'))
            if tags:
                self.start_tag = gr.tag_to_python(tags[0])
                print(f'Tag key: {self.start_tag.key} tag Value: {self.start_tag.value}\
                                   tag offset: {self.start_tag.offset}')

                self.current_packet = numpy.frombuffer(in0[self.start_tag.offset - first_item_offset:],
                                                       dtype=numpy.uint8)
                self.status = self.StreamStatus.LOOK_FOR_END

                consumed_items = self.current_packet.size + self.start_tag.offset - first_item_offset

        # If we found the start of the packet, look for the ending tag
        if self.status == self.StreamStatus.LOOK_FOR_END:
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

                self.current_packet = self.current_packet[:(self.end_tag.offset - self.start_tag.offset)]

                self.send_packet_msg()
                self.status = self.StreamStatus.IDLE

        # print(f'Board interface: consumed {in0_items_no} items')
        self.consume(0, in0_items_no)
        return 0
