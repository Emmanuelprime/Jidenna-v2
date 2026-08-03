#!/usr/bin/env python3
"""
Simple Robot Control - Just send v,w or s to stop
"""

import serial
import time
import sys

# ─── CONFIGURATION ──────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyUSB0'  # Windows: 'COM3', Linux: '/dev/ttyUSB0', Mac: '/dev/tty.usbserial-*'
BAUD_RATE = 115200

# ─── MAIN ────────────────────────────────────────────────────────────────────

def main():
    # Check if port provided as argument
    port = SERIAL_PORT
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    # Connect to robot
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"✅ Connected to {port}")
        print("\nCommands:")
        print("  v,w  - Set velocity (e.g., 0.2,0  or  0,0.5)")
        print("  s    - Stop")
        print("  q    - Quit")
        print("-" * 40)
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        sys.exit(1)
    
    try:
        while True:
            # Get user input
            cmd = input("> ").strip()
            
            if cmd.lower() == 'q':
                break
            elif cmd.lower() == 's':
                ser.write(b's\n')
                print("⏹️ Stopped")
            elif ',' in cmd:
                # Parse v,w command
                parts = cmd.split(',')
                if len(parts) == 2:
                    try:
                        v = float(parts[0])
                        w = float(parts[1])
                        # Send V command
                        ser.write(f'V{v},{w}\n'.encode())
                        print(f"▶️  v={v:.2f}, w={w:.2f}")
                    except ValueError:
                        print("❌ Invalid numbers. Use: v,w  (e.g., 0.2,0)")
                else:
                    print("❌ Use: v,w  (e.g., 0.2,0)")
            else:
                print("❌ Use: v,w  (e.g., 0.2,0)  or  s  or  q")
            
            # Show any response from robot
            time.sleep(0.1)
            while ser.in_waiting:
                line = ser.readline().decode().strip()
                if line and not line.startswith('CNT,'):
                    print(f"📩 {line}")
    
    except KeyboardInterrupt:
        print("\n\n👋 Exiting...")
    finally:
        ser.write(b's\n')  # Stop motors
        ser.close()
        print("🔌 Disconnected")

if __name__ == "__main__":
    main()