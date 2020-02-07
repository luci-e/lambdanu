#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: gfsk_tx
# Author: JoJo
# GNU Radio version: 3.9.0.0-git

from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from math import pi
import vlc

class gfsk_tx(gr.top_block):

    def __init__(self, parameter_0=0):
        gr.top_block.__init__(self, "gfsk_tx")

        ##################################################
        # Parameters
        ##################################################
        self.parameter_0 = parameter_0

        ##################################################
        # Variables
        ##################################################
        self.transmission_rate = transmission_rate = 200e3
        self.samp_rate = samp_rate = 4e6

        ##################################################
        # Blocks
        ##################################################
        self.vlc_vlc_packet_source_b_0 = vlc.vlc_packet_source_b('vlc_gr_server')
        self.vlc_vlc_board_interface_b_0 = vlc.vlc_board_interface_b('192.168.1.24', 7777, 6666, 256)
        self.blocks_unpack_k_bits_bb_0 = blocks.unpack_k_bits_bb(8)
        self.blocks_repeat_0 = blocks.repeat(gr.sizeof_float*1, int(samp_rate/transmission_rate*32))
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_ff(255)
        self.blocks_float_to_uchar_0 = blocks.float_to_uchar()
        self.blocks_char_to_float_0 = blocks.char_to_float(1, 1)



        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.vlc_vlc_board_interface_b_0, 'vlc_board_int_msg_out'), (self.vlc_vlc_packet_source_b_0, 'vlc_pkt_src_msg_in'))
        self.connect((self.blocks_char_to_float_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.blocks_float_to_uchar_0, 0), (self.vlc_vlc_board_interface_b_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_repeat_0, 0))
        self.connect((self.blocks_repeat_0, 0), (self.blocks_float_to_uchar_0, 0))
        self.connect((self.blocks_unpack_k_bits_bb_0, 0), (self.blocks_char_to_float_0, 0))
        self.connect((self.vlc_vlc_packet_source_b_0, 0), (self.blocks_unpack_k_bits_bb_0, 0))

    def get_parameter_0(self):
        return self.parameter_0

    def set_parameter_0(self, parameter_0):
        self.parameter_0 = parameter_0

    def get_transmission_rate(self):
        return self.transmission_rate

    def set_transmission_rate(self, transmission_rate):
        self.transmission_rate = transmission_rate
        self.blocks_repeat_0.set_interpolation(int(self.samp_rate/self.transmission_rate*32))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_repeat_0.set_interpolation(int(self.samp_rate/self.transmission_rate*32))


def argument_parser():
    parser = ArgumentParser()
    return parser


def main(top_block_cls=gfsk_tx, options=None):
    if options is None:
        options = argument_parser().parse_args()
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    tb.wait()


if __name__ == '__main__':
    main()
