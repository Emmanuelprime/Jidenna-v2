import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)

print("Robot Controller")
print("Commands:")
print("  V<left>,<right> - Set velocities (e.g., V0.2,0.2)")
print("  M<pwmL>,<pwmR>  - Manual PWM (e.g., M30,30)")
print("  S               - Stop")
print("  Q               - Quit")
print()

while True:
    cmd = input("> ").strip()
    
    if cmd.upper() == 'Q':
        break
    elif cmd.upper() == 'S':
        ser.write(b"S\n")
        print("Stopped")
    elif cmd.startswith('V') or cmd.startswith('v'):
        ser.write(f"{cmd}\n".encode())
        print(f"Sending: {cmd}")
    elif cmd.startswith('M') or cmd.startswith('m'):
        ser.write(f"{cmd}\n".encode())
        print(f"Sending: {cmd}")
    else:
        print("Unknown command")

ser.close()
print("Done")