#!/usr/bin/env python3
"""Simple Robot Controller - Send command for 5 seconds"""

import serial
import time

PORT = '/dev/ttyUSB0'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

print("Ready. Commands: V0.2,0  M30,30  S  Q")

try:
    while True:
        cmd = input("> ").strip()
        
        if cmd.upper() == 'Q':
            break
        
        print(f"Sending: {cmd} for 5 seconds...")
        
        # Keep sending the command every 0.5 seconds for 5 seconds
        start_time = time.time()
        while time.time() - start_time < 5:
            # Send the command again
            ser.write(f"{cmd}\n".encode())
            
            # Read and print any responses
            while ser.in_waiting:
                line = ser.readline().decode().strip()
                if line:
                    print(f"  {line}")
            
            time.sleep(0.5)  # Send every 500ms to prevent timeout
        
        # Stop after 5 seconds
        print("Stopping...")
        ser.write(b"S\n")
        time.sleep(0.5)
        
        # Print any final messages
        while ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                print(f"  {line}")

finally:
    ser.write(b"S\n")
    ser.close()
    print("\nDone")