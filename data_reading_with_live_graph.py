#live graph of data bitstream processed by pico

# Run under Local Python 3 - Thonny's Python

import serial, struct, time
import matplotlib.pyplot as plt

# ──────── Configuration ─────────────────────────────────────────────────────
PORT           = "/dev/cu.usbmodem1101"  # your Pico’s serial port
BAUDRATE       = 115200                 # USB-CDC ignores this, but pyserial needs it
PACKET_TIMEOUT = 20                     # seconds to wait for header+payload
NUM_SAMPLES    = 10                     # must match your main.py’s NUM_SAMPLES

# ──────── Open serial port ──────────────────────────────────────────────────
print(f"→ Opening {PORT}")
ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
ser.reset_input_buffer()

# ──────── Prepare live Matplotlib plot ───────────────────────────────────────
plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot([], [], "o-", label="Byte-1 edges")
line2, = ax.plot([], [], "s-", label="Byte-2 edges")
ax.set_xlim(0, NUM_SAMPLES-1)
ax.set_ylim(0, 1)             # will autoscale after first packet
ax.set_xlabel("Second")
ax.set_ylabel("Edge count")
ax.set_title("Live Pico edge-counter")
ax.legend()
fig.canvas.draw()

print("Listening for 10-second packets…")
print("Trigger each new packet by unplug/replug the Pico (normal power cycle).")

# ──────── Main loop ─────────────────────────────────────────────────────────
try:
    while True:
        # 1) Wait for 4-byte header
        t0 = time.time()
        while ser.in_waiting < 4:
            if time.time() - t0 > PACKET_TIMEOUT:
                raise RuntimeError("Timeout waiting for packet header")
            time.sleep(0.05)
        raw_cnt = ser.read(4)
        n, = struct.unpack("<I", raw_cnt)
        if n != NUM_SAMPLES:
            print(f"Got {n} samples (expected {NUM_SAMPLES})")

        # 2) Read payload
        expected = n * 4
        buf = bytearray()
        while len(buf) < expected:
            if time.time() - t0 > PACKET_TIMEOUT:
                raise RuntimeError(f"Timeout waiting payload {len(buf)}/{expected}")
            chunk = ser.read(expected - len(buf))
            if chunk:
                buf.extend(chunk)
        ser.readline()  # discard newline if any

        # 3) Unpack
        vals = struct.unpack(f"<{n}I", buf)
        edges1 = [(v >> 16) & 0xFFFF for v in vals]
        edges2 = [ v        & 0xFFFF for v in vals]

        # 4) Update plot
        line1.set_data(range(n), edges1)
        line2.set_data(range(n), edges2)
        ax.relim(); ax.autoscale_view()
        fig.canvas.draw(); fig.canvas.flush_events()

        print(f"[{time.strftime('%H:%M:%S')}] edges1={edges1}")

except KeyboardInterrupt:
    print("\nInterrupted by user; closing.")
finally:
    ser.close()
    plt.ioff()
    plt.show()

###############

#37 MHz square wave test


# import time
# import matplotlib.pyplot as plt
# 
# # ──────── Configuration ─────────────────────────────────────────────────────
# NUM_SAMPLES = 10        # points per trace
# PERIOD_S     = 10       # seconds between full trace updates
# FREQ_HZ      = 37000000     # 37 MHz
# EDGES_PER_SEC = FREQ_HZ  # assuming we count only rising edges
# 
# # ──────── Prepare live Matplotlib plot ───────────────────────────────────────
# plt.ion()
# fig, ax = plt.subplots()
# line1, = ax.plot([], [], "o-", label="Byte-1 edges")
# line2, = ax.plot([], [], "s-", label="Byte-2 edges")
# ax.set_xlim(0, NUM_SAMPLES-1)
# ax.set_ylim(0, EDGES_PER_SEC * 1.1)  # room for scaling
# ax.set_xlabel("Second")
# ax.set_ylabel("Edge count")
# ax.set_title("Simulated Pico edge-counter (1 kHz square wave)")
# ax.legend()
# fig.canvas.draw()
# 
# print(f"Simulating a 100 kHz square wave ({EDGES_PER_SEC} edges/sec)")
# 
# # ──────── Main loop ─────────────────────────────────────────────────────────
# try:
#     while True:
#         # simulate 10 seconds of 1-second measurements
#         edges1 = [EDGES_PER_SEC] * NUM_SAMPLES
#         edges2 = [EDGES_PER_SEC] * NUM_SAMPLES
# 
#         # update the plot
#         line1.set_data(range(NUM_SAMPLES), edges1)
#         line2.set_data(range(NUM_SAMPLES), edges2)
#         ax.relim()
#         ax.autoscale_view()
#         fig.canvas.draw()
#         fig.canvas.flush_events()
# 
#         print(f"[{time.strftime('%H:%M:%S')}] Simulated edges = {EDGES_PER_SEC} per second")
# 
#         # wait for the next cycle
#         time.sleep(PERIOD_S)
# 
# except KeyboardInterrupt:
#     print("\nSimulation stopped by user.")
# finally:
#     plt.ioff()
#     plt.show()



#Probabilistic Signal (100 KHz) test
# import time, random
# import matplotlib.pyplot as plt
# 
# # ──────── Configuration ─────────────────────────────────────────────────────
# NUM_SAMPLES   = 10        # points per trace
# PERIOD_S      = 10        # seconds between full trace updates
# FREQ_HZ       = 100_000  # 100 KHz signal
# PROB_UP       = 0.7       # probability of rising edge
# PROB_DOWN     = 0.4       # probability of falling edge
# 
# # ──────── Prepare live Matplotlib plot ───────────────────────────────────────
# plt.ion()
# fig, ax = plt.subplots()
# line1, = ax.plot([], [], "o-", label="Byte-1 edges (rising)")
# line2, = ax.plot([], [], "s-", label="Byte-2 edges (falling)")
# ax.set_xlim(0, NUM_SAMPLES-1)
# #ax.set_ylim(0, FREQ_HZ * 1.0)
# ax.set_xlabel("Second")
# ax.set_ylabel("Edge count")
# ax.set_title("Simulated Probabilistic 100 KHz Signal")
# ax.legend()
# fig.canvas.draw()
# 
# print(f"Simulating a noisy 37 MHz signal with:")
# print(f"  • 70% rising-edge chance (0→1)")
# print(f"  • 40% falling-edge chance (1→0)")
# 
# # ──────── Main loop ─────────────────────────────────────────────────────────
# try:
#     while True:
#         edges1 = []  # rising edges
#         edges2 = []  # falling edges
# 
#         for _ in range(NUM_SAMPLES):  # simulate 10 seconds
#             rising = 0
#             falling = 0
#             state = 0  # start at low
# 
#             for _ in range(FREQ_HZ):  # simulate each cycle in 1 second
#                 if state == 0 and random.random() < PROB_UP:
#                     state = 1
#                     rising += 1
#                 elif state == 1 and random.random() < PROB_DOWN:
#                     state = 0
#                     falling += 1
# 
#             edges1.append(rising)
#             edges2.append(falling)
# 
#         # Update plot
#         line1.set_data(range(NUM_SAMPLES), edges1)
#         line2.set_data(range(NUM_SAMPLES), edges2)
#         ax.relim(); ax.autoscale_view()
#         fig.canvas.draw(); fig.canvas.flush_events()
# 
#         print(f"[{time.strftime('%H:%M:%S')}] Rising: {edges1[-1]}, Falling: {edges2[-1]}")
# 
#         time.sleep(PERIOD_S)
# 
# except KeyboardInterrupt:
#     print("\nSimulation stopped by user.")
# finally:
#     plt.ioff()
#     plt.show()