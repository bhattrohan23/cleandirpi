# cleandirpi
DiRPi system using a Raspberry Pi Pico 2 chip.

MicroPython and PIO are used to read square waves using Pulse Width Modulation.

The DiRPi sends two bytes of data to the Pico 2, using 8 GPIO pins for 8 bits each. 

GPIO pins 2-9 (inclusive) take in one byte, and pins 14-21 (inclusive) take in the other byte.

There is minimal delay in reading the number of edges of the incoming waveform, indicating accurate reading. The code works up to 37 MHz. Luckily, the DiRPi can be calibrated to work at 20 MHz, which the Pico can handle. The process of reading the number of edges is done in main.py, which is automatically run every time the Pico is connected to Thonny or another compatible IDE. Edge counts are determined by looking at the number of rising edges over one-second intervals.

The read signal is then send to a Python script via USB serial. It receives 10 second packets of edge counts (timing can be toggled easily), and a matplotlib graph is updated every 10 seconds with the edge counts over the 10 second period.
