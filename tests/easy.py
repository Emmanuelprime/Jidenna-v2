import serial
import time
import sys
import math

try:
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=115200,
        timeout=1
    )
    print(f"Serial port opened: {ser.port}")
except Exception as e:
    print(f"Failed to open serial port: {e}")
    sys.exit(1)

print("Waiting for ESP32 to initialize (including MPU calibration)...")
time.sleep(5)

ser.reset_input_buffer()
ser.reset_output_buffer()
print("Buffers cleared")

# Test parameters
linear_velocity = 0.2
duration = 10.0
command_interval = 0.05

# PID controller for heading correction - CONSERVATIVE GAINS
target_heading = None
kp_heading = 1.5       # Reduced from 5.0 to prevent oscillation
ki_heading = 0.3       # Reduced from 0.5 for slower integral buildup
kd_heading = 0.3       # Increased for better damping

# PID variables
integral_error = 0.0
previous_error = 0.0
previous_time = time.time()

# Maximum angular velocity correction (rad/s)
max_correction = 0.3   # Reduced from 0.5 to prevent aggressive corrections
min_correction = 0.01  # Slightly larger deadband to prevent hunting

# Moving average filter for MPU readings
mpu_history = []
mpu_filter_size = 5

print("="*80)
print("STARTING STRAIGHT-LINE DRIVING WITH MPU CORRECTION")
print("="*80)
print(f"Linear velocity: {linear_velocity} m/s")
print(f"Duration: {duration} seconds")
print(f"PID Gains - Kp: {kp_heading}, Ki: {ki_heading}, Kd: {kd_heading}")
print(f"Max correction: {max_correction} rad/s")
print(f"Command interval: {command_interval*1000}ms")
print(f"MPU filter size: {mpu_filter_size}")
print("Waiting for first MPU reading...")
print("-"*80)

start_time = time.time()
last_command_time = 0
data_count = 0
command_count = 0
data_points = []

def filter_mpu(new_value):
    """Apply moving average filter to MPU readings"""
    global mpu_history
    mpu_history.append(new_value)
    if len(mpu_history) > mpu_filter_size:
        mpu_history.pop(0)
    return sum(mpu_history) / len(mpu_history)

def compute_heading_correction(current_heading, dt):
    """Compute angular velocity correction to maintain straight line"""
    global integral_error, previous_error, target_heading
    
    # Calculate heading error (wrap to [-pi, pi])
    heading_error = target_heading - current_heading
    while heading_error > math.pi:
        heading_error -= 2 * math.pi
    while heading_error < -math.pi:
        heading_error += 2 * math.pi
    
    # Apply deadband to prevent hunting
    if abs(heading_error) < 0.02:  # 1.15 degrees
        heading_error = 0.0
        integral_error *= 0.9  # Decay integral when in deadband
    
    # PID calculation
    integral_error += heading_error * dt
    derivative_error = (heading_error - previous_error) / dt if dt > 0 else 0
    
    # Limit integral windup
    integral_error = max(-0.5, min(0.5, integral_error))
    
    # Compute individual PID components
    p_term = kp_heading * heading_error
    i_term = ki_heading * integral_error
    d_term = kd_heading * derivative_error
    
    # Total correction
    correction = p_term + i_term + d_term
    
    # Apply deadband
    if abs(correction) < min_correction:
        correction = 0.0
    
    # Limit correction
    if abs(correction) > max_correction:
        correction = max(-max_correction, min(max_correction, correction))
    
    # Update previous error
    previous_error = heading_error
    
    return correction, heading_error, p_term, i_term, d_term

try:
    # Send initial command to start data flow
    initial_command = f"V{linear_velocity},0\n"
    ser.write(initial_command.encode())
    print(f"Sent initial command: {initial_command.strip()}")
    
    while (time.time() - start_time) < duration:
        current_time = time.time()
        
        # Check for incoming data
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                
                if line and ',' in line:
                    values = line.split(',')
                    if len(values) == 6:
                        data_count += 1
                        dt = current_time - previous_time
                        previous_time = current_time
                        
                        # Parse data
                        x = float(values[0])
                        y = float(values[1])
                        theta = float(values[2])
                        vl = float(values[3])
                        vr = float(values[4])
                        mpu_z_raw = float(values[5])
                        
                        # Apply moving average filter to MPU
                        mpu_z = filter_mpu(mpu_z_raw)
                        
                        # Set target heading on first data point (after filter has data)
                        if target_heading is None and len(mpu_history) == mpu_filter_size:
                            target_heading = mpu_z
                            print(f"\nTarget heading set to: {target_heading:.4f} rad ({target_heading*180/math.pi:.2f} deg)")
                            print("-"*80)
                            continue
                        
                        # Only compute correction if target is set
                        if target_heading is not None:
                            # Store data point
                            data_points.append((current_time - start_time, x, y, mpu_z, theta, vl, vr))
                            
                            # Compute heading correction
                            angular_correction, heading_error, p_term, i_term, d_term = compute_heading_correction(mpu_z, dt)
                            
                            # Send corrected command
                            if (current_time - last_command_time) >= command_interval:
                                command = f"V{linear_velocity},{angular_correction}\n"
                                ser.write(command.encode())
                                command_count += 1
                                last_command_time = current_time
                            
                            # Print every 10th data point
                            if data_count % 10 == 1:
                                print(f"\n[Data #{data_count}] t={current_time-start_time:.2f}s")
                                print(f"  Position: x={x:.3f}m, y={y:.3f}m")
                                print(f"  MPU Z (filtered): {mpu_z:.4f} rad ({mpu_z*180/math.pi:.2f} deg)")
                                print(f"  Heading error: {heading_error:.4f} rad ({heading_error*180/math.pi:.2f} deg)")
                                print(f"  PID - P: {p_term:.4f}, I: {i_term:.4f}, D: {d_term:.4f}")
                                print(f"  Correction: {angular_correction:.4f} rad/s")
                                print(f"  Wheel speeds: vl={vl:.3f}, vr={vr:.3f}")
                                print(f"  Command: V={linear_velocity}, omega={angular_correction:.4f}")
                    
            except ValueError as e:
                print(f"Error parsing data: {e}")
            except Exception as e:
                print(f"Error reading data: {e}")
        
        # If no data, send neutral command
        elif (current_time - last_command_time) >= command_interval:
            command = f"V{linear_velocity},0\n"
            ser.write(command.encode())
            command_count += 1
            last_command_time = current_time
        
        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n\nInterrupted by user!")

finally:
    # Stop the robot
    print("\n" + "="*80)
    print("STOPPING ROBOT")
    print("="*80)
    
    # Send stop command multiple times
    for _ in range(5):
        ser.write(b"V0,0\n")
        time.sleep(0.1)
    
    ser.close()
    
    elapsed_time = time.time() - start_time
    
    # Analyze data
    print(f"\nSession Summary:")
    print(f"  Duration: {elapsed_time:.2f} seconds")
    print(f"  Data points received: {data_count}")
    print(f"  Commands sent: {command_count}")
    
    if data_count > 0:
        print(f"  Data rate: {data_count/elapsed_time:.1f} Hz")
        print(f"  Target heading: {target_heading:.4f} rad" if target_heading else "  Target heading: Not set")
        print(f"  Final heading error: {previous_error:.4f} rad ({previous_error*180/math.pi:.2f} deg)")
        
        # Calculate drift metrics
        if len(data_points) > 1:
            initial_y = data_points[0][2]
            final_y = data_points[-1][2]
            total_drift = final_y - initial_y
            print(f"  Total Y drift: {total_drift:.3f} m over {elapsed_time:.1f}s")
            print(f"  Drift rate: {total_drift/elapsed_time*100:.1f} cm/s")
            
            # Calculate average wheel speed difference
            speed_diffs = [abs(d[5] - d[6]) for d in data_points]
            avg_speed_diff = sum(speed_diffs) / len(speed_diffs) if speed_diffs else 0
            print(f"  Average wheel speed difference: {avg_speed_diff:.3f} m/s")
    else:
        print("  No data received - check connections")
    
    print("\nRobot stopped.")