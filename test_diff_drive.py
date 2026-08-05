import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)

print("Robot Controller")
print("Commands:")
print("  D<V>,<W>         - Differential drive (V=m/s, W=rad/s)")
print("  V<left>,<right>  - Set individual wheel velocities (m/s)")
print("  M<pwmL>,<pwmR>   - Manual PWM (e.g., M30,30)")
print("  P                - Print current pose")
print("  S                - Stop")
print("  Q                - Quit")
print()
print("Examples:")
print("  D0.2,0.0   - Drive forward at 0.2 m/s")
print("  D0.0,0.5   - Turn in place at 0.5 rad/s")
print("  D0.2,0.3   - Forward while turning right")
print()

while True:
    cmd = input("> ").strip()
    
    if cmd.upper() == 'Q':
        break
    elif cmd.upper() == 'S':
        ser.write(b"S\n")
        print("Stopped")
    elif cmd.upper() == 'P':
        ser.write(b"P\n")
        # Read and print response
        time.sleep(0.1)
        while ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                print(line)
    elif cmd.startswith('D') or cmd.startswith('d'):
        # Differential drive: D<V>,<W>
        ser.write(f"{cmd}\n".encode())
        print(f"Sending: {cmd}")
    elif cmd.startswith('V') or cmd.startswith('v'):
        # Individual wheel velocities: V<left>,<right>
        ser.write(f"{cmd}\n".encode())
        print(f"Sending: {cmd}")
    elif cmd.startswith('M') or cmd.startswith('m'):
        # Manual PWM: M<pwmL>,<pwmR>
        ser.write(f"{cmd}\n".encode())
        print(f"Sending: {cmd}")
    else:
        print("Unknown command. Use D, V, M, P, S, or Q")

ser.close()
print("Done")