#!/usr/bin/env python3
"""Simple Robot Controller - Send command for 5 seconds"""

import serial
import time

PORT = '/dev/ttyUSB0'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)  # Reduced timeout
time.sleep(2)
ser.reset_input_buffer()

print("Ready. Commands: V0.2,0  V0,0.5  Q")

def send_command(ser, cmd):
    """Send command to Arduino"""
    ser.write(f"{cmd}\n".encode())
    ser.flush()  # Ensure command is sent immediately

try:
    while True:
        cmd = input("> ").strip()
        
        if cmd.upper() == 'Q':
            break
        
        # Validate command format
        if not (cmd.startswith('V') or cmd.startswith('v')):
            print("Invalid command. Use format: V<linear>,<angular> (e.g., V0.2,0)")
            continue
        
        print(f"Sending: {cmd} for 5 seconds...")
        
        # Keep sending the command for 5 seconds
        start_time = time.time()
        next_send_time = start_time
        
        while time.time() - start_time < 5:
            current_time = time.time()
            
            # Send command every 100ms (10Hz - well within 500ms timeout)
            if current_time >= next_send_time:
                send_command(ser, cmd)
                next_send_time = current_time + 0.1  # Send every 100ms
            
            # Read and print any responses (non-blocking)
            while ser.in_waiting:
                line = ser.readline().decode().strip()
                if line:
                    print(f"  {line}")
            
            # Small sleep to prevent CPU overload
            time.sleep(0.01)
        
        # Stop after 5 seconds
        print("Stopping...")
        send_command(ser, "V0,0")
        time.sleep(0.5)
        
        # Print any final messages
        while ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                print(f"  {line}")

except KeyboardInterrupt:
    print("\nInterrupted by user")
    
finally:
    send_command(ser, "V0,0")  # Stop the robot
    time.sleep(0.1)
    ser.close()
    print("Done")