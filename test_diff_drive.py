import serial
import time
import threading

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)

# Global flag for reading thread
reading = True

def read_serial():
    """Background thread to continuously read and print serial data"""
    while reading:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                # Check if it's a debug line (starts with "Heading" or contains "error")
                if "Heading" in line or "error" in line or "Correction" in line:
                    print(f"[DEBUG] {line}")
                elif "," in line and not line.startswith("Pose"):
                    # Odometry data - show with timestamp
                    parts = line.split(',')
                    if len(parts) >= 8:
                        try:
                            x = float(parts[0])
                            y = float(parts[1])
                            theta = float(parts[2])
                            vl = float(parts[3])
                            vr = float(parts[4])
                            target_vl = float(parts[6])
                            target_vr = float(parts[7])
                            print(f"[ODOM] x={x:.3f}, y={y:.3f}, theta={theta:.3f}°, vl={vl:.3f}, vr={vr:.3f}, target=({target_vl:.3f},{target_vr:.3f})")
                        except:
                            print(f"[RAW] {line}")
                else:
                    print(f"[MSG] {line}")
        time.sleep(0.01)

# Start reading thread
read_thread = threading.Thread(target=read_serial, daemon=True)
read_thread.start()

print("="*60)
print("ROBOT CONTROLLER WITH DEBUG")
print("="*60)
print("Commands:")
print("  D<V>,<W>         - Differential drive (V=m/s, W=rad/s)")
print("  V<left>,<right>  - Set individual wheel velocities (m/s)")
print("  M<pwmL>,<pwmR>   - Manual PWM (e.g., M30,30)")
print("  H                - Toggle heading lock")
print("  P                - Print current pose")
print("  Y                - Print MPU data")
print("  S                - Stop")
print("  Q                - Quit")
print()
print("Examples:")
print("  D0.2,0.0   - Drive forward at 0.2 m/s")
print("  D0.0,0.5   - Turn in place at 0.5 rad/s")
print("  D0.2,0.3   - Forward while turning right")
print("="*60)
print()

try:
    while True:
        cmd = input("> ").strip()
        
        if cmd.upper() == 'Q':
            break
        elif cmd.upper() == 'S':
            ser.write(b"S\n")
            print("[CMD] Stopped")
        elif cmd.upper() == 'P':
            ser.write(b"P\n")
            print("[CMD] Requesting pose...")
        elif cmd.upper() == 'Y':
            ser.write(b"Y\n")
            print("[CMD] Requesting MPU data...")
        elif cmd.upper() == 'H':
            ser.write(b"H\n")
            print("[CMD] Toggling heading lock...")
        elif cmd.startswith('D') or cmd.startswith('d'):
            ser.write(f"{cmd}\n".encode())
            print(f"[CMD] Sending differential drive: {cmd}")
        elif cmd.startswith('V') or cmd.startswith('v'):
            ser.write(f"{cmd}\n".encode())
            print(f"[CMD] Sending wheel velocities: {cmd}")
        elif cmd.startswith('M') or cmd.startswith('m'):
            ser.write(f"{cmd}\n".encode())
            print(f"[CMD] Sending manual PWM: {cmd}")
        else:
            print("[ERROR] Unknown command. Use D, V, M, H, P, Y, S, or Q")
            
except KeyboardInterrupt:
    print("\n[INFO] Interrupted by user")

finally:
    reading = False
    ser.close()
    print("[INFO] Done")