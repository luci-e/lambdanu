Directory structure

/
- docs : docs for the various parts used in the project
-- buildroot
-- f7
-- mp1
-- tex
- source
-- firmware : contains the firmware for the boards
--- F7 : Nucleo F767ZI
---- trx : transceiver with RF
---- trxcopyfrom : CubeMX project
---- trxnoRF : trx with the RF part removed
--- MP1 : STM32MP157ADK1
-- gnuradio :
--- gr-graphs : graphs for the gnuradio-companion
--- gr-vlc : module created with gr_modtool
-- VLCStack : the rest of the VLC stack


Initializing the boards

Copy source/firmware/F7/trxnoRF/BUILD/NUCLEO_F767ZI/GCC_ARM/trxnoRF.bin to the board mounted as mass storage.

Connect to the board via your favorite serial monitor at 2000000 bauds

The board base address is 192.168.1.22, you can increase it by pressing the blue button on the board as soon as they get reset. The final address will be written on the serial.

The default block size for data transfer is 256 bytes
The default data port is 6666
The default cmd port is 7777 

If you want to recompile the code for the boards you'll need the mbed-os toolchain. See:
https://os.mbed.com/docs/mbed-os/v5.15/tools/developing-mbed-cli.html


Testing out the boards

After connecting the board you can bypass gnuradio by using the python tool in VLCStack/testLib. You should be able to switch between the modes of the board as well as transmitting waves and receiving ADC data.

An example test script is testBoard.py

GNURadio

-- TODO --

cmake ../ -DPYTHON_EXECUTABLE='path/to/python>3.5/executable'
