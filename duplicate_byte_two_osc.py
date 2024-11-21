#duplicate of 2 byte osc program
# mezcard + oscilloscope, reads byte 1 and serializes byte 2 on GPIO10
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio
import time

# --- Define GPIO pins for Byte 1 and Byte 2 ---
BYTE1_PINS = list(range(2, 10))    # GPIO2 to GPIO9 (8 pins)
BYTE2_PINS = list(range(14, 22))   # GPIO14 to GPIO21 (8 contiguous pins)

# --- Configure Pins for Byte 1 ---
for pin_num in BYTE1_PINS:
    Pin(pin_num, Pin.IN, Pin.PULL_DOWN)

# --- Configure Pins for Byte 2 ---
for pin_num in BYTE2_PINS:
    Pin(pin_num, Pin.IN, Pin.PULL_DOWN)

# --- Configure GPIO pin for serial output ---
serial_out_pin = Pin(12, Pin.OUT)  # GPIO12 will serialize Byte 2

# --- PIO Program for Edge Detection on Byte 1 ---
@asm_pio()
def edge_detect_byte1():
    pull()                     # Pull initial previous sample from FIFO
    mov(x, osr)                # Move to x register
    label("loop")
    in_(pins, 8)               # Read 8 bits from in_base (GPIO2-9) into ISR
    jmp(x_not_y, "changed")    # If current != previous, jump to 'changed'
    jmp("loop")                # Else, continue looping
    label("changed")
    push(block)                # Push current sample to FIFO
    mov(x, isr)                # Update previous sample
    jmp("loop")                # Continue looping

# --- PIO Program for Edge Detection on Byte 2 ---
@asm_pio()
def edge_detect_byte2():
    pull()                     # Pull initial previous sample from FIFO
    mov(x, osr)                # Move to x register
    label("loop")
    in_(pins, 8)               # Read 8 bits from in_base (GPIO14-21) into ISR
    jmp(x_not_y, "changed")    # If current != previous, jump to 'changed'
    jmp("loop")                # Else, continue looping
    label("changed")
    push(block)                # Push current sample to FIFO
    mov(x, isr)                # Update previous sample
    jmp("loop")                # Continue looping

# --- PIO Program to Serialize and Output Byte 2 ---
@asm_pio(
    out_init=PIO.OUT_LOW,
    out_shiftdir=PIO.SHIFT_RIGHT,  # Shift out from LSB to MSB
    autopull=False,
)
def serialize_byte2():
    wrap_target()
    
    set (12, 1) 
    set (12, 0) 
    
    set (12, 1) 
    set (12, 0)
    wrap()

# --- Initialize State Machine for Byte 1 Edge Detection ---
sm_byte1 = StateMachine(
    0,                              # State Machine index
    edge_detect_byte1,              # PIO program
    freq=150_000_000,               # Clock frequency
    in_base=Pin(BYTE1_PINS[0]),      # Starting pin for input (GPIO2)
)
sm_byte1.put(0)  # Initialize previous sample to 0

# --- Initialize State Machine for Byte 2 Edge Detection ---
sm_byte2 = StateMachine(
    1,                              # State Machine index
    edge_detect_byte2,              # PIO program
    freq=150_000_000,               # Clock frequency
    in_base=Pin(BYTE2_PINS[0]),      # Starting pin for input (GPIO14)
)
sm_byte2.put(0)  # Initialize previous sample to 0

# --- Initialize State Machine for Serializing Byte 2 ---
sm_serialize = StateMachine(
    2,                              # State Machine index
    serialize_byte2,                # PIO program
    freq=150_000_000,               # Clock frequency
    in_base=Pin(BYTE2_PINS[0]),      # Starting pin for input (GPIO14)
    out_base=serial_out_pin,         # Output pin (GPIO10)
)

# --- Start State Machines ---
sm_byte1.active(1)
sm_byte2.active(1)
sm_serialize.active(1)  # Start serializing Byte 2 on GPIO10

# --- Main Loop for Edge Counting ---
try:
    while True:
        edge_count_byte1 = 0
        edge_count_byte2 = 0
        prev_sample_byte1 = 0
        prev_sample_byte2 = 0
        start_time = time.ticks_ms()

        # Count edges for 1 second
        while time.ticks_diff(time.ticks_ms(), start_time) < 1000:
            # Read from State Machine 0 (Byte 1)
            while sm_byte1.rx_fifo():
                data = sm_byte1.get() & 0xFF  # Get 8 bits
                # Detect rising edges by comparing with previous sample
                edges = bin((~prev_sample_byte1) & data).count('1')
                edge_count_byte1 += edges
                prev_sample_byte1 = data

            # Read from State Machine 1 (Byte 2)
            while sm_byte2.rx_fifo():
                data = sm_byte2.get() & 0xFF  # Get 8 bits
                # Detect rising edges by comparing with previous sample
                edges = bin((~prev_sample_byte2) & data).count('1')
                edge_count_byte2 += edges
                prev_sample_byte2 = data

        # Display results
        print(f"Edges Detected on Byte 1 in last second: {edge_count_byte1}")
        print(f"Edges Detected on Byte 2 in last second: {edge_count_byte2}\n")

except KeyboardInterrupt:
    print("Monitoring stopped.")
    sm_byte1.active(0)
    sm_byte2.active(0)
    sm_serialize.active(0)