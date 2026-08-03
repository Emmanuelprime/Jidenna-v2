#!/usr/bin/env python3
"""
Ultra-simple Robot Controller
"""

import serial
import time
import sys
import threading

PORT = '/dev/ttyUSB0'  # Change to your port
BAUD = 115200

def reader_thread(ser):
    """Read serial data without printing CNT messages"""
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            # Only print non-telemetry messages
            if not line.startswith("CNT,") and line:
                print(f"\n>> {line}")
                print("Enter command (v,w or s): ", end='', flush=True)

# Connect
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

# Start reader
thread = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
thread.start()

print("\nRobot Controller Ready!")
print("Commands: v,w (e.g., 0.5,0) or 's' to stop, 'q' to quit\n")

try:
    while True:
        cmd = input("Enter command (v,w or s): ").strip()
        
        if cmd == 'q':
            break
        elif cmd == 's':
            ser.write(b's\n')
            print("Sent: STOP")
        elif ',' in cmd:
            ser.write(f"V{cmd}\n".encode())
            print(f"Sent: V{cmd}")
        else:
            print("Invalid command!")
finally:
    ser.write(b's\n')
    time.sleep(0.2)
    ser.close()
    print("\nDisconnected")