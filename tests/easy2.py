import serial
import time
import sys

# Configuration
PORT = 'COM19'  # Change this to your port (e.g., '/dev/ttyUSB0' on Linux)
BAUD = 115200

def send_command(ser, v, w):
    """Send V,W command to robot"""
    cmd = f"V{v:.2f},{w:.2f}\n"
    ser.write(cmd.encode())
    print(f"Sent: {cmd.strip()}")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # Wait for ESP32 to reset
        print(f"Connected to {PORT}")
        print("Commands: forward, turn, stop, exit")
        print("Examples: V0.20,0.00  V0.00,0.50  V0.15,-0.30\n")
        
        while True:
            cmd = input("Enter command: ").strip()
            
            if cmd.lower() == 'exit':
                send_command(ser, 0, 0)
                break
                
            if cmd.lower() == 'stop':
                send_command(ser, 0, 0)
                continue
                
            if cmd.lower() == 'forward':
                print("Moving forward for 10 seconds...")
                send_command(ser, 0.20, 0.00)
                time.sleep(10)
                send_command(ser, 0, 0)
                print("Stopped")
                continue
                
            if cmd.lower() == 'turn':
                print("Turning for 10 seconds...")
                send_command(ser, 0.00, 0.50)
                time.sleep(10)
                send_command(ser, 0, 0)
                print("Stopped")
                continue
            
            # Parse manual V,W command
            if cmd.startswith('V'):
                send_command(ser, 0, 0)  # Stop first
                time.sleep(0.1)
                
                # Parse the command
                parts = cmd[1:].split(',')
                if len(parts) == 2:
                    v = float(parts[0])
                    w = float(parts[1])
                    print(f"Sending for 10 seconds...")
                    send_command(ser, v, w)
                    time.sleep(10)
                    send_command(ser, 0, 0)
                    print("Stopped")
                else:
                    print("Invalid format. Use: V0.20,0.00")
            else:
                print("Commands: forward, turn, stop, V0.20,0.00, exit")
                
    except serial.SerialException as e:
        print(f"Error: {e}")
        print(f"Check port '{PORT}' and try again")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()