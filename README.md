# cleandirpi
DiRPi system using a Raspberry Pi Pico 2 chip.

MicroPython and PIO are used to read square waves using Pulse Width Modulation.

The DiRPi sends a byte of data to the Pico 2, using 8 GPIO pins for 8 bits each. 

GPIO pins 2-9 (inclusive) take in one byte, and pins 14-21 (inclusive) take in the other byte.

There is minimal delay in reading the number of edges of the incoming waveform, indicating accurate reading. The code works up to 37 MHz. Luckily, the DiRPi can be calibrated to work at 20 MHz, which the Pico can handle.
