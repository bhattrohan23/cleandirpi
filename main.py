#main program that is already run on Pico firmware that reads the bitstream of the signal sent as two bytes into the proper pins
# main_single_cdc.py  – Pico-2 edge-counter, streams recurring 10-s packets
#                      on *the* CDC port (same port as the REPL)

from machine import Pin
from rp2     import PIO, StateMachine, asm_pio
import sys, time

# ---------------------------------------------------------------------------
# 1.  Data port  (single CDC) ────────────────────────────────────────────────
# ---------------------------------------------------------------------------
data_port = sys.stdout.buffer
time.sleep(0.2)      # let host enumerate

# ---------------------------------------------------------------------------
# 2.  GPIO definitions
# ---------------------------------------------------------------------------
BYTE1_PINS = list(range(2, 10))      # GPIO2-9
BYTE2_PINS = list(range(14, 22))     # GPIO14-21
for p in BYTE1_PINS: Pin(p, Pin.IN, Pin.PULL_DOWN)
for p in BYTE2_PINS: Pin(p, Pin.IN, Pin.PULL_DOWN)

serial_out_pin = Pin(12, Pin.OUT)    # optional serializer (unused here)

# ---------------------------------------------------------------------------
# 3.  PIO programs
# ---------------------------------------------------------------------------
@asm_pio()
def edge_detect_byte():
    pull(); mov(x, osr)
    label("loop")
    in_(pins, 8)
    jmp(x_not_y, "changed")
    jmp("loop")
    label("changed")
    push(block)
    mov(x, isr)
    jmp("loop")

@asm_pio(out_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_RIGHT, autopull=False)
def serialize_byte2():
    wrap_target()
    set(12, 1); set(12, 0)
    set(12, 1); set(12, 0)
    wrap()

# ---------------------------------------------------------------------------
# 4.  State-machine setup
# ---------------------------------------------------------------------------
sm1 = StateMachine(0, edge_detect_byte, freq=150_000_000,
                   in_base=Pin(BYTE1_PINS[0]))
sm2 = StateMachine(1, edge_detect_byte, freq=150_000_000,
                   in_base=Pin(BYTE2_PINS[0]))
sm1.put(0); sm2.put(0)
sm1.active(1); sm2.active(1)

sm_ser = StateMachine(2, serialize_byte2, freq=150_000_000,
                      in_base=Pin(BYTE2_PINS[0]), out_base=serial_out_pin)
sm_ser.active(1)

# ---------------------------------------------------------------------------
# 5.  Continuous capture and transmit
# ---------------------------------------------------------------------------
NUM_SAMPLES = 10  # 10 × 1-second windows → 10-second packet

try:
    while True:
        data_buffer = [0] * NUM_SAMPLES
        for i in range(NUM_SAMPLES):
            e1 = e2 = 0
            p1 = p2 = 0
            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) < 1000:
                while sm1.rx_fifo():
                    d = sm1.get() & 0xFF
                    e1 += bin((~p1) & d).count('1')
                    p1 = d
                while sm2.rx_fifo():
                    d = sm2.get() & 0xFF
                    e2 += bin((~p2) & d).count('1')
                    p2 = d
            data_buffer[i] = (e1 << 16) | (e2 & 0xFFFF)

        # --- Stream binary packet ---
        data_port.write(NUM_SAMPLES.to_bytes(4, 'little'))
        for val in data_buffer:
            data_port.write(val.to_bytes(4, 'little'))
        data_port.write(b'\n')  # optional: helps if reading in terminal
        data_port.flush()

        # brief pause before next cycle
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Cancelled by user.")

finally:
    sm1.active(0)
    sm2.active(0)
    sm_ser.active(0)
