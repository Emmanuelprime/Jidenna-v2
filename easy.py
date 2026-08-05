#!/usr/bin/env python3
"""Ultra Minimal Robot Controller"""

import serial
import time

PORT = 'C'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

print("Ready. Send raw commands: V0.2,0  M30,30  S  Z  Q")

try:
    while True:
        cmd = input("> ").strip()
        
        if cmd.upper() == 'Q':
            break
        
        # Send directly - whatever you type goes straight to ESP32
        ser.write(f"{cmd}\n".encode())
        
        # Print responses
        time.sleep(0.1)
        while ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                print(f"  {line}")

finally:
    ser.write(b's\n')
    ser.close()
    print("\nDone")