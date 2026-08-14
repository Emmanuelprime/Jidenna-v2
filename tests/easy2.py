import serial
import time
import sys
import math
import matplotlib.pyplot as plt
from collections import deque

if len(sys.argv) > 1:
    linear_velocity = float(sys.argv[1])
else:
    linear_velocity = 0.2

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    timeout=1
)

time.sleep(5)
ser.reset_input_buffer()
ser.write(b"V0,0\n")
ser.flush()

print(f"Starting straight-line driving at {linear_velocity} m/s...")
print("Press Ctrl+C to stop")
print("-"*80)

command_interval = 0.05
last_command_time = 0
start_time = time.time()
data_count = 0

fused_heading = None
last_robot_timestamp = None
gyro_bias = 0.0
gyro_bias_samples = []
gyro_bias_calibration_count = 50
is_calibrating = True

filter_time_constant = 0.5

kp_heading = 5.5
ki_heading = 1.0
kd_heading = 0.15
target_heading = None
integral_error = 0.0
max_correction = 0.8
min_correction = 0.005

time_data = []
theta_data = []
mpu_angle_data = []
gyro_rate_data = []
gyro_rate_corrected_data = []
fused_heading_data = []
correction_data = []
p_term_data = []
i_term_data = []
d_term_data = []
heading_error_data = []
vl_data = []
vr_data = []

def unwrap_angles(angles):
    unwrapped = []
    prev = None
    for angle in angles:
        if prev is None:
            unwrapped.append(angle)
        else:
            while angle - prev > math.pi:
                angle -= 2 * math.pi
            while angle - prev < -math.pi:
                angle += 2 * math.pi
            unwrapped.append(angle)
        prev = angle
    return unwrapped

def fuse_theta(odometry_theta, gyro_rate_rad, dt):
    global fused_heading
    
    if fused_heading is None:
        fused_heading = odometry_theta
        return fused_heading
    
    gyro_heading = fused_heading + gyro_rate_rad * dt
    
    alpha = filter_time_constant / (filter_time_constant + dt)
    
    fused_heading = alpha * gyro_heading + (1 - alpha) * odometry_theta
    
    while fused_heading > math.pi:
        fused_heading -= 2 * math.pi
    while fused_heading < -math.pi:
        fused_heading += 2 * math.pi
    
    return fused_heading

def compute_correction(current_heading, gyro_rate_rad, dt):
    global integral_error, target_heading
    
    if target_heading is None:
        target_heading = current_heading
        return 0.0, 0.0, 0.0, 0.0, 0.0
    
    heading_error = target_heading - current_heading
    while heading_error > math.pi:
        heading_error -= 2 * math.pi
    while heading_error < -math.pi:
        heading_error += 2 * math.pi
    
    if abs(heading_error) < 0.01:
        heading_error = 0.0
        integral_error *= 0.95
    
    integral_error += heading_error * dt
    integral_error = max(-2.0, min(2.0, integral_error))
    
    p_term = kp_heading * heading_error
    i_term = ki_heading * integral_error
    d_term = -kd_heading * gyro_rate_rad
    
    correction = p_term + i_term + d_term
    
    if abs(correction) < min_correction:
        correction = 0.0
    
    if abs(correction) > max_correction:
        correction = max(-max_correction, min(max_correction, correction))
    
    return correction, heading_error, p_term, i_term, d_term

def plot_filter_data():
    if len(time_data) == 0:
        print("No data to plot")
        return
    
    theta_unwrapped = unwrap_angles(theta_data)
    mpu_unwrapped = unwrap_angles(mpu_angle_data)
    fused_unwrapped = unwrap_angles(fused_heading_data)
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    axes[0].plot(time_data, theta_unwrapped, 'b-', label='Odometry Theta', linewidth=1)
    axes[0].plot(time_data, mpu_unwrapped, 'r-', label='MPU Angle', linewidth=1)
    axes[0].plot(time_data, fused_unwrapped, 'g-', label='Fused Heading', linewidth=2)
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Heading (rad)')
    axes[0].set_title('Heading Fusion Results')
    axes[0].legend()
    axes[0].grid(True)
    
    axes[1].plot(time_data, gyro_rate_data, 'r-', label='Raw Gyro', linewidth=0.5, alpha=0.5)
    axes[1].plot(time_data, gyro_rate_corrected_data, 'b-', label='Bias-Corrected Gyro', linewidth=2)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Gyro Rate (rad/s)')
    axes[1].set_title('Gyroscope Z Rate - Raw vs Bias-Corrected')
    axes[1].legend()
    axes[1].grid(True)
    
    plt.tight_layout()
    plt.show()

def plot_controller_data():
    if len(time_data) == 0:
        print("No data to plot")
        return
    
    fused_unwrapped = unwrap_angles(fused_heading_data)
    
    fig, axes = plt.subplots(4, 1, figsize=(12, 12))
    
    axes[0].plot(time_data, fused_unwrapped, 'b-', label='Fused Heading', linewidth=2)
    if target_heading is not None:
        axes[0].axhline(y=target_heading, color='r', linestyle='--', label=f'Target: {target_heading:.3f} rad', linewidth=2)
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Heading (rad)')
    axes[0].set_title('Controller - Setpoint vs Actual Heading')
    axes[0].legend()
    axes[0].grid(True)
    
    axes[1].plot(time_data, heading_error_data, 'r-', linewidth=1.5)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Heading Error (rad)')
    axes[1].set_title('Controller - Heading Error')
    axes[1].grid(True)
    
    axes[2].plot(time_data, p_term_data, 'r-', label='P Term', linewidth=1)
    axes[2].plot(time_data, i_term_data, 'g-', label='I Term', linewidth=1)
    axes[2].plot(time_data, d_term_data, 'b-', label='D Term (gyro)', linewidth=1)
    axes[2].plot(time_data, correction_data, 'k-', label='Control Signal', linewidth=2)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Correction (rad/s)')
    axes[2].set_title('Controller - PID Components and Control Signal')
    axes[2].legend()
    axes[2].grid(True)
    
    axes[3].plot(time_data, vl_data, 'b-', label='Left Wheel', linewidth=1)
    axes[3].plot(time_data, vr_data, 'r-', label='Right Wheel', linewidth=1)
    axes[3].set_xlabel('Time (s)')
    axes[3].set_ylabel('Speed (m/s)')
    axes[3].set_title('Wheel Speeds')
    axes[3].legend()
    axes[3].grid(True)
    
    plt.tight_layout()
    plt.show()

try:
    while True:
        current_time = time.time()
        elapsed = current_time - start_time
        
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            if line and ',' in line:
                values = line.split(',')
                if len(values) == 8:  # Changed from 7 to 8
                    data_count += 1
                    
                    x = float(values[0])
                    y = float(values[1])
                    theta = float(values[2])
                    vl = float(values[3])
                    vr = float(values[4])
                    mpu_angle_deg = float(values[5])
                    gyro_rate_deg = float(values[6])
                    robot_timestamp = float(values[7])  # milliseconds
                    
                    mpu_angle_rad = mpu_angle_deg * math.pi / 180.0
                    gyro_rate_rad = gyro_rate_deg * math.pi / 180.0
                    
                    if is_calibrating:
                        gyro_bias_samples.append(gyro_rate_rad)
                        if len(gyro_bias_samples) >= gyro_bias_calibration_count:
                            gyro_bias = sum(gyro_bias_samples) / len(gyro_bias_samples)
                            is_calibrating = False
                            print(f"\nGyro bias calibrated: {gyro_bias:.6f} rad/s")
                            print("-"*80)
                        last_robot_timestamp = robot_timestamp
                        continue
                    
                    gyro_rate_corrected = gyro_rate_rad - gyro_bias
                    
                    if last_robot_timestamp is not None:
                        dt = (robot_timestamp - last_robot_timestamp) / 1000.0
                        if dt <= 0 or dt > 1.0:
                            dt = 0.01
                    else:
                        dt = 0.01
                    last_robot_timestamp = robot_timestamp
                    
                    current_fused = fuse_theta(theta, gyro_rate_corrected, dt)
                    
                    correction, heading_error, p_term, i_term, d_term = compute_correction(current_fused, gyro_rate_corrected, dt)
                    
                    if (current_time - last_command_time) >= command_interval:
                        command = f"V{linear_velocity},{correction}\n"
                        ser.write(command.encode())
                        last_command_time = current_time
                    
                    time_data.append(elapsed)
                    theta_data.append(theta)
                    mpu_angle_data.append(mpu_angle_rad)
                    gyro_rate_data.append(gyro_rate_rad)
                    gyro_rate_corrected_data.append(gyro_rate_corrected)
                    fused_heading_data.append(current_fused)
                    correction_data.append(correction)
                    p_term_data.append(p_term)
                    i_term_data.append(i_term)
                    d_term_data.append(d_term)
                    heading_error_data.append(heading_error)
                    vl_data.append(vl)
                    vr_data.append(vr)
                    
                    print(f"t={elapsed:.2f}s, "
                          f"dt={dt*1000:.0f}ms, "
                          f"theta={theta:.3f}, "
                          f"fused={current_fused:.3f}, "
                          f"error={heading_error:.3f}, "
                          f"corr={correction:.3f}, "
                          f"vl={vl:.2f}, vr={vr:.2f}")
        
        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n\nData collection interrupted by user.")
    print(f"Total data points collected: {data_count}")
    print(f"Gyro bias: {gyro_bias:.6f} rad/s")
    print(f"Average dt: {sum([(time_data[i+1]-time_data[i]) for i in range(len(time_data)-1)])/max(len(time_data)-1,1)*1000:.1f}ms")
    
    if target_heading is not None:
        print(f"Target heading: {target_heading:.4f} rad ({target_heading*180/math.pi:.2f} deg)")
    if fused_heading is not None:
        print(f"Final fused heading: {fused_heading:.4f} rad ({fused_heading*180/math.pi:.2f} deg)")
    
    print("\nPlotting filter data...")
    plot_filter_data()
    
    print("\nPlotting controller data...")
    plot_controller_data()

finally:
    print("Stopping robot...")
    ser.write(b"V0,0\n")
    time.sleep(0.5)
    ser.write(b"V0,0\n")
    time.sleep(0.1)
    ser.close()
    print("Robot stopped. Serial connection closed.")