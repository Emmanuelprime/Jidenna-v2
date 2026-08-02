import serial
import time
import threading
import sys
import atexit

class RobotController:
    def __init__(self, port='/dev/ttyUSB0', baud=115200, timeout=0.1):
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            self.ser.flushInput()
            self.ser.flushOutput()
        except serial.SerialException as e:
            print(f"Error opening serial port {port}: {e}")
            sys.exit(1)
            
        self.running = False
        self.telemetry = {}
        self.telemetry_thread = None
        self.lock = threading.Lock()
        self.last_heartbeat = time.time()
        
        # Register cleanup on exit
        atexit.register(self.cleanup)
        
    def send_velocity(self, v, w):
        """Send velocity command: v in m/s, w in rad/s"""
        try:
            cmd = f"V{v:.3f},{w:.3f}\n"
            self.ser.write(cmd.encode())
            self.ser.flush()
            print(f"Sent: {cmd.strip()}")
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
            
    def send_velocity_command(self, cmd):
        """Parse and send velocity command from string (for interactive mode)"""
        try:
            # Remove 'V' if present
            if cmd.upper().startswith('V'):
                cmd = cmd[1:]
            
            # Parse v,w
            parts = cmd.split(',')
            if len(parts) == 2:
                v = float(parts[0].strip())
                w = float(parts[1].strip())
                self.send_velocity(v, w)
            else:
                print("Invalid format. Use: V<v>,<w> (e.g., V0.5,0.0)")
        except ValueError as e:
            print(f"Invalid number format: {e}")
        except Exception as e:
            print(f"Error parsing command: {e}")
        
    def emergency_stop(self):
        """Emergency stop the robot"""
        try:
            self.ser.write(b"STOP\n")
            self.ser.flush()
            print("Emergency stop sent")
        except serial.SerialException as e:
            print(f"Error sending emergency stop: {e}")
        
    def reset_robot(self):
        """Reset the ESP32"""
        try:
            self.ser.write(b"RESET\n")
            self.ser.flush()
            print("Reset command sent")
        except serial.SerialException as e:
            print(f"Error sending reset: {e}")
        
    def calibrate_imu(self):
        """Calibrate the IMU"""
        try:
            self.ser.write(b"CALIBRATE\n")
            self.ser.flush()
            print("Calibration command sent")
            print("Keep robot still for 3 seconds...")
        except serial.SerialException as e:
            print(f"Error sending calibration: {e}")
        
    def get_status(self):
        """Get robot status"""
        try:
            self.ser.write(b"STATUS\n")
            self.ser.flush()
            time.sleep(0.1)
            
            # Read all available lines
            responses = []
            start_time = time.time()
            while time.time() - start_time < 0.5:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode().strip()
                    if line:
                        responses.append(line)
                        if line.startswith("STATUS"):
                            return line
                time.sleep(0.01)
            
            # Return last response if no STATUS found
            return responses[-1] if responses else None
        except serial.SerialException as e:
            print(f"Error getting status: {e}")
            return None
    
    def read_telemetry(self):
        """Read continuous telemetry stream"""
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode().strip()
                    if line:
                        self.last_heartbeat = time.time()
                        
                        if line.startswith("CNT"):
                            parts = line.split(',')
                            if len(parts) >= 13:  # Updated for state field
                                with self.lock:
                                    self.telemetry = {
                                        'time': int(parts[1]),
                                        'left_speed': float(parts[2]),
                                        'right_speed': float(parts[3]),
                                        'linear': float(parts[4]),
                                        'omega': float(parts[5]),
                                        'filtered_omega': float(parts[6]),
                                        'yaw': float(parts[7]),
                                        'x': float(parts[8]),
                                        'y': float(parts[9]),
                                        'left_pwm': int(parts[10]),
                                        'right_pwm': int(parts[11]),
                                        'state': int(parts[12]) if len(parts) > 12 else 0
                                    }
                        elif line.startswith("ACK"):
                            print(f"ACK: {line}")
                        elif line.startswith("WARN"):
                            print(f"WARNING: {line}")
                        elif line.startswith("READY"):
                            print("Robot ready!")
                else:
                    # Check for connection timeout
                    if time.time() - self.last_heartbeat > 3.0:
                        # Only print once every 10 seconds to avoid spam
                        if int(time.time()) % 10 == 0:
                            print("Warning: No data from robot...")
                        self.last_heartbeat = time.time()
                        
            except serial.SerialException as e:
                print(f"Serial error in telemetry thread: {e}")
                break
            except UnicodeDecodeError:
                # Skip invalid characters
                continue
            except Exception as e:
                print(f"Unexpected error in telemetry thread: {e}")
                continue
                
            time.sleep(0.01)  # Small delay to prevent CPU spinning
    
    def start_telemetry(self):
        """Start telemetry reading thread"""
        if self.telemetry_thread and self.telemetry_thread.is_alive():
            print("Telemetry thread already running")
            return
            
        self.running = True
        self.telemetry_thread = threading.Thread(target=self.read_telemetry, daemon=False)
        self.telemetry_thread.start()
        print("Telemetry thread started")
        time.sleep(0.5)  # Wait for thread to start
    
    def stop_telemetry(self):
        """Stop telemetry reading thread"""
        self.running = False
        if self.telemetry_thread and self.telemetry_thread.is_alive():
            self.telemetry_thread.join(timeout=2.0)
            print("Telemetry thread stopped")
    
    def get_telemetry(self):
        """Get latest telemetry data"""
        with self.lock:
            return self.telemetry.copy()
    
    def wait_for_telemetry(self, timeout=5.0):
        """Wait for first telemetry packet"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.lock:
                if self.telemetry:
                    return True
            time.sleep(0.1)
        return False
    
    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        self.running = False
        if self.telemetry_thread and self.telemetry_thread.is_alive():
            self.telemetry_thread.join(timeout=1.0)
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                # Send stop before closing
                self.ser.write(b"STOP\n")
                self.ser.flush()
                time.sleep(0.1)
                self.ser.close()
                print("Serial port closed")
            except:
                pass
    
    def close(self):
        """Close the connection"""
        self.cleanup()
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()
        return False


# ─── TEST FUNCTIONS ──────────────────────────────────────────────────────────

def test_basic_movement(robot):
    """Test basic movement commands"""
    print("\n=== Testing Basic Movement ===")
    
    # Wait for telemetry
    if robot.wait_for_telemetry():
        print("Telemetry connection established")
    else:
        print("Warning: No telemetry received")
    
    # Forward
    print("\nMoving forward at 0.3 m/s for 3 seconds...")
    robot.send_velocity(0.3, 0.0)
    time.sleep(3)
    
    # Get position
    telemetry = robot.get_telemetry()
    print(f"Position: x={telemetry.get('x', 0):.3f}, y={telemetry.get('y', 0):.3f}")
    print(f"Yaw: {telemetry.get('yaw', 0):.2f} degrees")
    
    # Stop
    print("\nStopping...")
    robot.send_velocity(0, 0)
    time.sleep(1)

def test_turning(robot):
    """Test turning commands"""
    print("\n=== Testing Turning ===")
    
    # Turn
    print("\nTurning at 0.5 rad/s for 2 seconds...")
    robot.send_velocity(0.0, 0.5)
    time.sleep(2)
    
    # Get yaw
    telemetry = robot.get_telemetry()
    print(f"Yaw: {telemetry.get('yaw', 0):.2f} degrees")
    
    # Stop
    print("\nStopping...")
    robot.send_velocity(0, 0)
    time.sleep(1)

def test_square_path(robot):
    """Test square path (requires accurate odometry)"""
    print("\n=== Testing Square Path ===")
    
    # Move forward
    print("\nMoving forward 0.4m...")
    robot.send_velocity(0.2, 0.0)
    time.sleep(2)
    robot.send_velocity(0, 0)
    time.sleep(0.5)
    
    # Turn 90 degrees
    print("Turning 90 degrees...")
    robot.send_velocity(0.0, 0.5)
    time.sleep(1.57)  # 90 degrees at 0.5 rad/s = π/2
    robot.send_velocity(0, 0)
    time.sleep(0.5)
    
    # Move forward again
    print("Moving forward 0.4m...")
    robot.send_velocity(0.2, 0.0)
    time.sleep(2)
    robot.send_velocity(0, 0)
    
    telemetry = robot.get_telemetry()
    print(f"Final position: x={telemetry.get('x', 0):.3f}, y={telemetry.get('y', 0):.3f}")

def interactive_mode(robot):
    """Interactive command mode"""
    print("\n=== Interactive Mode ===")
    print("Commands:")
    print("  V<v>,<w> - Set velocity (e.g., V0.5,0.0 or v0.5,0.0)")
    print("  STOP - Emergency stop")
    print("  STATUS - Get robot status")
    print("  CALIBRATE - Calibrate IMU")
    print("  RESET - Reset ESP32")
    print("  QUIT - Exit")
    print("\nExamples:")
    print("  V0.3,0.0  - Move forward at 0.3 m/s")
    print("  V0.0,0.5  - Turn at 0.5 rad/s")
    print("  V-0.2,0.0 - Move backward at 0.2 m/s")
    print()
    
    while True:
        try:
            cmd = input("> ").strip()
            if not cmd:
                continue
                
            cmd_upper = cmd.upper()
            
            if cmd_upper == "QUIT" or cmd_upper == "Q":
                break
            elif cmd_upper == "STOP":
                robot.emergency_stop()
            elif cmd_upper == "STATUS":
                status = robot.get_status()
                if status:
                    print(status)
                else:
                    print("No status received")
            elif cmd_upper == "CALIBRATE":
                robot.calibrate_imu()
            elif cmd_upper == "RESET":
                robot.reset_robot()
            elif cmd_upper.startswith("V"):
                robot.send_velocity_command(cmd)
            else:
                print(f"Unknown command: {cmd}")
                print("Available commands: V<v>,<w>, STOP, STATUS, CALIBRATE, RESET, QUIT")
                
        except KeyboardInterrupt:
            print("\nInterrupted")
            break
        except Exception as e:
            print(f"Error: {e}")

# ─── MAIN ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Default port - change as needed
    PORT = "COM19"  # Windows example
    # PORT = "/dev/ttyUSB0"  # Linux example
    # PORT = "/dev/tty.usbserial-*"  # Mac example
    
    print("ESP32 Robot Controller")
    print("=" * 50)
    print(f"Connecting to {PORT}...")
    
    # Use context manager for automatic cleanup
    with RobotController(port=PORT, baud=115200) as robot:
        # Start telemetry
        robot.start_telemetry()
        time.sleep(1)  # Give time for connection
        
        # Wait for robot ready message
        if robot.wait_for_telemetry(timeout=3.0):
            print("Connected to robot!")
        else:
            print("Warning: No telemetry received. Check connection.")
        
        # Run tests
        try:
            # Uncomment the tests you want to run
            test_basic_movement(robot)
            # test_turning(robot)
            # test_square_path(robot)
            
            # Enter interactive mode
            interactive_mode(robot)
            
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"Error in main: {e}")
        finally:
            # Emergency stop on exit
            robot.emergency_stop()
            print("\nEmergency stop sent")
            time.sleep(0.5)
    
    print("Program ended")