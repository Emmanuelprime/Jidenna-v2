#!/usr/bin/env python3
"""Minimal Robot Controller"""

import serial
import time

PORT = '/dev/ttyUSB0'
BAUD = 115200

# Connect
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

print("Ready. Commands: v,0.2,0  m,30,30  s  z  q")

try:
    while True:
        cmd = input("> ").strip()
        
        if cmd == 'q':
            break
        elif cmd == 's':
            ser.write(b's\n')
        elif cmd == 'z':
            ser.write(b'z\n')
        elif cmd.startswith('v,') or cmd.startswith('V,'):
            ser.write(f"V{cmd[1:]}\n".encode())
        elif cmd.startswith('m,') or cmd.startswith('M,'):
            ser.write(f"M{cmd[1:]}\n".encode())
        else:
            print("?")
        
        # Print any response
        time.sleep(0.1)
        while ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                print(f"  {line}")

finally:
    ser.write(b's\n')
    ser.close()
    print("Done")