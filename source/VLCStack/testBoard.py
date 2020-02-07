#!/usr/bin/python3.7

from testTool import VLCtester
import numpy as np
import matplotlib.pylab as plt

board_addr = '192.168.1.22'

# Generate a wave sample at 32 spc , repeated 4 times
test_wave =  ( ( np.tile( np.sin(  np.linspace(-np.pi, np.pi, 32)) , 4 )  + 1 ) * 255 / 2).astype(np.uint8)
 
t = VLCtester(board_address = board_addr)

t.send_set_mode_command('idle')
t.send_set_mode_command('tx')
t.buffer_vlc_wave(test_wave)
t.transmit_last_wave()
t.send_set_mode_command('rx')

adc = numpy.ndarray(0, dtype=np.uint8)

while i < 10:
	adc.append(np.frombuffer(get_data_block, dtype=np.uint8))

plt.plot(adc)
plt.show()

t.send_set_mode_command('idle')
