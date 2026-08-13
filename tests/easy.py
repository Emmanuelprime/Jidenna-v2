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

print("="*80)
print("STARTING STRAIGHT-LINE DRIVING WITH MPU CORRECTION")
print("="*80)
print(f"Target heading: {target_heading} rad (0 = straight ahead)")
print(f"Linear velocity: {linear_velocity} m/s")
print(f"Duration: {duration} seconds")
print(f"PID Gains - Kp: {kp_heading}, Ki: {ki_heading}, Kd: {kd_heading}")
print(f"Max correction: ±{max_correction} rad/s")
print(f"Command interval: {command_interval*1000}ms")
print("-"*80)

start_time = time.time()
command_interval = 0.1  # Send command every 100ms
last_command_time = 0
data_count = 0
command_count = 0

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
    
    # Compute individual PID components for debugging
    p_term = kp_heading * heading_error
    i_term = ki_heading * integral_error
    d_term = kd_heading * derivative_error
    
    # Total correction
    correction = p_term + i_term + d_term
    
    # Apply deadband
    if abs(correction) < min_correction:
        correction = 0.0
        print(f"  ↳ Correction in deadband (< {min_correction}), setting to 0")
    
    # Limit correction
    if abs(correction) > max_correction:
        print(f"  ↳ Correction limited from {correction:.3f} to {max_correction * (1 if correction > 0 else -1):.3f}")
        correction = max(-max_correction, min(max_correction, correction))
    
    # Update previous error
    previous_error = heading_error
    
    return correction, heading_error, p_term, i_term, d_term

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
                    data_count += 1
                    
                    # Parse data
                    x = float(values[0])
                    y = float(values[1])
                    theta = float(values[2])
                    vl = float(values[3])
                    vr = float(values[4])
                    mpu_z = float(values[5])
                    
                    print(f"\n[Data #{data_count}] t={current_time-start_time:.2f}s")
                    print(f"  Position: x={x:.3f}m, y={y:.3f}m")
                    print(f"  Odometry theta: {theta:.3f} rad")
                    print(f"  Wheel speeds: vl={vl:.3f} m/s, vr={vr:.3f} m/s")
                    print(f"  MPU Z angle: {mpu_z:.2f}° ({mpu_z*180/3.14159:.2f}°)")
                    
                    # Compute heading correction using MPU Z angle
                    angular_correction, heading_error, p_term, i_term, d_term = compute_heading_correction(mpu_z, dt)
                    
                    # Print correction details
                    print(f"  Heading error: {heading_error:.3f} rad ({heading_error*180/3.14159:.2f}°)")
                    print(f"  PID components - P: {p_term:.3f}, I: {i_term:.3f}, D: {d_term:.3f}")
                    print(f"  Angular correction: {angular_correction:.3f} rad/s")
                    
                    # Send corrected command
                    if (current_time - last_command_time) >= command_interval:
                        command = f"V{linear_velocity},{angular_correction}\n"
                        ser.write(command.encode())
                        command_count += 1
                        last_command_time = current_time
                        print(f"  → Sent command: {command.strip()}")
                        print(f"  → Command #{command_count}: V={linear_velocity} m/s, ω={angular_correction:.3f} rad/s")
                    else:
                        time_to_next = command_interval - (current_time - last_command_time)
                        print(f"  → Waiting {time_to_next*1000:.0f}ms before next command")
        
        # If no data received, send command with last known correction
        elif (current_time - last_command_time) >= command_interval:
            # Use last computed correction (or 0 if none)
            command = f"V{linear_velocity},0\n"
            ser.write(command.encode())
            command_count += 1
            last_command_time = current_time
            print(f"\n[No data] Sending neutral command: {command.strip()}")
        
        time.sleep(0.01)  # Small delay to prevent CPU overload

except KeyboardInterrupt:
    print("\n\n⚠️  Interrupted by user!")

finally:
    # Stop the robot
    print("\n" + "="*80)
    print("STOPPING ROBOT")
    print("="*80)
    ser.write(b"V0,0\n")
    time.sleep(0.1)
    ser.close()
    
    elapsed_time = time.time() - start_time
    print(f"\nSession Summary:")
    print(f"  Duration: {elapsed_time:.2f} seconds")
    print(f"  Data points received: {data_count}")
    print(f"  Commands sent: {command_count}")
    print(f"  Final heading error: {previous_error:.3f} rad ({previous_error*180/3.14159:.2f}°)")
    print(f"  Final MPU Z angle: {mpu_z:.2f}°" if 'mpu_z' in locals() else "  No MPU data received")
    print("\nRobot stopped successfully.")