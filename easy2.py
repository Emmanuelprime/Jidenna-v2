#!/usr/bin/env python3
import serial
import time
import sys

PORT = '/dev/ttyUSB0'
BAUD = 115200

if len(sys.argv) > 1:
    PORT = sys.argv[1]

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
print(f"Connected to {PORT}")
print("Commands: v,w  |  s=stop  |  q=quit")

try:
    while True:
        cmd = input("> ").strip()
        if cmd == 'q':
            break
        elif cmd == 's':
            ser.write(b's\n')
            print("Stopped")
        elif ',' in cmd:
            parts = cmd.split(',')
            if len(parts) == 2:
                try:
                    v, w = float(parts[0]), float(parts[1])
                    ser.write(f'V{v},{w}\n'.encode())
                    print(f"v={v:.2f}, w={w:.2f}")
                except:
                    print("Invalid numbers")
        # Clear serial buffer (ignore all data)
        while ser.in_waiting:
            ser.read(ser.in_waiting)
finally:
    ser.write(b's\n')
    ser.close()
    print("Disconnected")