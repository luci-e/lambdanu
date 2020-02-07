#!/usr/bin/python3.7
import socket
import numpy

class VLCtester:

    def __init__(self, board_address = '192.168.1.22', cmd_port = 7777, data_port = 6666, block_size = 256):
        self.board_address = board_address
        self.cmd_port = cmd_port
        self.data_port = data_port
        self.block_size = block_size

        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_addr = (board_address, cmd_port)
        self.data_addr = (board_address, data_port)

    def send_set_mode_command(mode):
        modes = {'idle': [0, 0], 'tx': [0, 1], 'rx': [0, 2], 'rf': [0, 3]}
        print(f'Sending set {mode} command to board!')
        buffer_command = numpy.array(modes[mode], dtype=numpy.uint32)
        self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)
        _reply = self.sock_cmd.recv(8)

        if mode == 'rx':
            buffer_command = numpy.array([1], dtype=numpy.uint32)
            self.sock_data.sendto(buffer_command.tobytes(), self.data_addr)

    def buffer_vlc_wave(wave: numpy.ndarray):
        print(f'Sending wave to store into the buffer')

        buffer_command = numpy.array([1, wave.size], dtype=numpy.uint32)
        self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)
        _reply = self.sock_cmd.recv(8)

        sent_data = 0

        while sent_data < wave.size:
            # It's ok to send more than the size of the array since it gets clipped anyway
            self.sock_data.sendto(wave[sent_data:sent_data + self.block_size].tobytes(),
                                  self.data_addr)
            sent_data += self.block_size

            _reply = self.sock_cmd.recv(8)

    def transmit_last_wave(self):
        print('Transmitting last buffered wave')

        buffer_command = numpy.array([4, 0], dtype=numpy.uint32)
        self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)

        reply = numpy.frombuffer(self.sock_cmd.recv(8), dtype=numpy.uint32)
        if reply[1] == 0:
            _reply = self.sock_cmd.recv(8)

    def get_data_block(self):
        return self.sock_data.recv(self.block_size)

    def send_rf_pkt(length):
        # Generate random packet of given length and send it to the board
        payload = f'{length}'
        payload = ('0' * (length - len(payload)) ) + payload

        print(f'Sending {payload} to rf')

        payload = '000hello'

        buffer_command = numpy.array([2, length], dtype=numpy.uint32)
        self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)
        _reply = self.sock_cmd.recv(8)

        self.sock_data.sendto(payload.encode('ascii'), self.data_addr)
        _reply = self.sock_cmd.recv(8)


    def receive_rf_pkt(self):
        buffer_command = numpy.array([3, 0], dtype=numpy.uint32)
        self.sock_cmd.sendto(buffer_command.tobytes(), self.cmd_addr)
        reply = self.sock_cmd.recv(8)
        r = numpy.frombuffer(reply, dtype=numpy.uint32)

        if r[1] != 0:
            data = self.sock_data.recv(8)
            print(f'Received {data}')
        else:
            print('No data available')

