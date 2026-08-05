import serial
import time
import threading
import sys

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)

# Global flag for reading thread
reading = True
last_odom_time = 0

def read_serial():
    """Background thread to continuously read and print serial data"""
    global last_odom_time
    while reading:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                # Check if it's a debug line (starts with "Heading" or contains "error" or "Correction")
                if "Heading" in line or "error" in line or "Correction" in line:
                    print(f"[DEBUG] {line}")
                elif "Yaw" in line or "Gyro" in line:
                    print(f"[MPU] {line}")
                elif "Pose" in line:
                    print(f"[POSE] {line}")
                elif "Differential" in line or "Target" in line or "Stopped" in line:
                    print(f"[CMD] {line}")
                elif "," in line and not line.startswith("Pose"):
                    # Odometry data - continuous stream
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
                            
                            # Print every 100ms (10Hz)
                            now = time.time()
                            if now - last_odom_time >= 0.1:
                                print(f"[ODOM] x={x:7.3f}, y={y:7.3f}, theta={theta:7.3f}°, vl={vl:5.3f}, vr={vr:5.3f}, target=({target_vl:5.3f},{target_vr:5.3f})")
                                last_odom_time = now
                        except:
                            print(f"[RAW] {line}")
                else:
                    print(f"[MSG] {line}")
        time.sleep(0.001)  # Small delay to prevent CPU hogging

# Start reading thread
read_thread = threading.Thread(target=read_serial, daemon=True)
read_thread.start()

print("="*70)
print("ROBOT CONTROLLER WITH CONTINUOUS DATA STREAM")
print("="*70)
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
print("="*70)
print()
print("Waiting for data... (continuous stream will appear below)")
print("-"*70)

try:
    while True:
        # Get user input without blocking the read thread
        try:
            # Use non-blocking input with timeout
            import select
            if select.select([sys.stdin], [], [], 0.1)[0]:
                cmd = sys.stdin.readline().strip()
                
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
                elif cmd:
                    print("[ERROR] Unknown command. Use D, V, M, H, P, Y, S, or Q")
        except:
            pass
            
except KeyboardInterrupt:
    print("\n[INFO] Interrupted by user")

finally:
    reading = False
    ser.close()
    print("[INFO] Done")