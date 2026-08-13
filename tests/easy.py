import serial
import time

# Configure serial connection (adjust port as needed)
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Change this to your port
    baudrate=115200,
    timeout=1
)

# Give the Arduino time to initialize
time.sleep(2)

# Flush any existing data
ser.reset_input_buffer()

# Test parameters
linear_velocity = 0.2    # m/s (desired forward speed)
duration = 10.0           # seconds

# PID controller for heading correction
target_heading = 0.0     # Target heading angle (0 = straight ahead)
kp_heading = 2.0         # Proportional gain for heading correction
ki_heading = 0.1         # Integral gain
kd_heading = 0.05        # Derivative gain

# PID variables
integral_error = 0.0
previous_error = 0.0
previous_time = time.time()

# Maximum angular velocity correction (rad/s)
max_correction = 1.0
# Minimum angular velocity correction (deadband)
min_correction = 0.01

print("Starting straight-line driving with MPU correction...")
print("Format: x, y, theta, vl, vr, mpu_z, correction, angular_cmd")

start_time = time.time()
command_interval = 0.1  # Send command every 100ms
last_command_time = 0

def compute_heading_correction(current_heading, dt):
    """Compute angular velocity correction to maintain straight line"""
    global integral_error, previous_error
    
    # Calculate heading error (wrap to [-pi, pi])
    heading_error = target_heading - current_heading
    while heading_error > 3.14159:
        heading_error -= 2 * 3.14159
    while heading_error < -3.14159:
        heading_error += 2 * 3.14159
    
    # PID calculation
    integral_error += heading_error * dt
    derivative_error = (heading_error - previous_error) / dt if dt > 0 else 0
    
    # Compute correction (angular velocity)
    correction = (kp_heading * heading_error + 
                  ki_heading * integral_error + 
                  kd_heading * derivative_error)
    
    # Apply deadband
    if abs(correction) < min_correction:
        correction = 0.0
    
    # Limit correction
    correction = max(-max_correction, min(max_correction, correction))
    
    # Update previous error
    previous_error = heading_error
    
    return correction, heading_error

try:
    while (time.time() - start_time) < duration:
        current_time = time.time()
        dt = current_time - previous_time
        previous_time = current_time
        
        # Read incoming data
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            if line and ',' in line:  # Validate data line
                values = line.split(',')
                if len(values) == 6:
                    # Parse data
                    x = float(values[0])
                    y = float(values[1])
                    theta = float(values[2])
                    vl = float(values[3])
                    vr = float(values[4])
                    mpu_z = float(values[5])
                    
                    # Compute heading correction using MPU Z angle
                    angular_correction, heading_error = compute_heading_correction(mpu_z, dt)
                    
                    # Print data with correction
                    print(f"x={x:.3f}, y={y:.3f}, theta={theta:.3f}, "
                          f"vl={vl:.3f}, vr={vr:.3f}, mpu_z={mpu_z:.2f}, "
                          f"error={heading_error:.3f}, correction={angular_correction:.3f}")
                    
                    # Send corrected command
                    if (current_time - last_command_time) >= command_interval:
                        command = f"V{linear_velocity},{angular_correction}\n"
                        ser.write(command.encode())
                        last_command_time = current_time
        
        # If no data received, send command with last known correction
        elif (current_time - last_command_time) >= command_interval:
            # Use last computed correction (or 0 if none)
            command = f"V{linear_velocity},0\n"
            ser.write(command.encode())
            last_command_time = current_time
        
        time.sleep(0.01)  # Small delay to prevent CPU overload

finally:
    # Stop the robot
    ser.write(b"V0,0\n")
    time.sleep(0.1)
    ser.close()
    print("\nData collection complete. Robot stopped.")
    print(f"Final heading error: {previous_error:.3f} rad")