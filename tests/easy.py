import serial
import time
import sys
import math
import matplotlib.pyplot as plt
from collections import deque

ser = serial.Serial(
    port='COM19',
    baudrate=115200,
    timeout=1
)

time.sleep(5)

ser.reset_input_buffer()

linear_velocity = 0.2

print("Starting straight-line driving with heading control...")
print("Format: x, y, theta, vl, vr, mpu_angle, gyro_rate, fused_heading, correction")
print("Press Ctrl+C to stop")
print("-"*80)

command_interval = 0.015
last_command_time = 0
start_time = time.time()
data_count = 0

fused_heading = None
last_time = time.time()
last_gyro_rate = 0.0
gyro_weight = 0.98
odometry_weight = 0.02

gyro_filter_size = 5
gyro_filter_buffer = deque(maxlen=gyro_filter_size)

kp_heading = 2.0
ki_heading = 0.3
kd_heading = 0.1
target_heading = None
integral_error = 0.0
previous_error = 0.0
max_correction = 0.5
min_correction = 0.01

time_data = []
theta_data = []
mpu_angle_data = []
gyro_rate_data = []
gyro_rate_filtered_data = []
fused_heading_data = []
correction_data = []
p_term_data = []
i_term_data = []
d_term_data = []
heading_error_data = []

def filter_gyro(gyro_rate):
    gyro_filter_buffer.append(gyro_rate)
    return sum(gyro_filter_buffer) / len(gyro_filter_buffer)

def fuse_theta(odometry_theta, gyro_rate_deg, dt):
    global fused_heading, last_gyro_rate

    gyro_rate_rad = gyro_rate_deg * math.pi / 180.0

    if fused_heading is None:
        fused_heading = odometry_theta
        last_gyro_rate = gyro_rate_rad
        return fused_heading

    gyro_heading = fused_heading + gyro_rate_rad * dt
    
    fused_heading = gyro_weight * gyro_heading + odometry_weight * odometry_theta

    while fused_heading > math.pi:
        fused_heading -= 2 * math.pi
    while fused_heading < -math.pi:
        fused_heading += 2 * math.pi
    
    last_gyro_rate = gyro_rate_rad
    
    return fused_heading

def compute_correction(current_heading, dt):
    global integral_error, previous_error, target_heading
    
    if target_heading is None:
        target_heading = current_heading
        return 0.0, 0.0, 0.0, 0.0, 0.0
    
    heading_error = target_heading - current_heading
    while heading_error > math.pi:
        heading_error -= 2 * math.pi
    while heading_error < -math.pi:
        heading_error += 2 * math.pi
    
    integral_error += heading_error * dt
    derivative_error = (heading_error - previous_error) / dt if dt > 0 else 0
    
    integral_error = max(-1.0, min(1.0, integral_error))
    
    p_term = kp_heading * heading_error
    i_term = ki_heading * integral_error
    d_term = kd_heading * derivative_error
    
    correction = p_term + i_term + d_term
    
    if abs(correction) < min_correction:
        correction = 0.0
    
    if abs(correction) > max_correction:
        correction = max(-max_correction, min(max_correction, correction))
    
    previous_error = heading_error
    
    return correction, heading_error, p_term, i_term, d_term

def plot_filter_data():
    if len(time_data) == 0:
        print("No data to plot")
        return
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    axes[0].plot(time_data, theta_data, 'b-', label='Odometry Theta', linewidth=1)
    axes[0].plot(time_data, mpu_angle_data, 'r-', label='MPU Angle', linewidth=1)
    axes[0].plot(time_data, fused_heading_data, 'g-', label='Fused Heading', linewidth=2)
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Heading (rad)')
    axes[0].set_title('Heading Fusion Results')
    axes[0].legend()
    axes[0].grid(True)
    
    axes[1].plot(time_data, gyro_rate_data, 'r-', label='Raw Gyro', linewidth=0.5, alpha=0.5)
    axes[1].plot(time_data, gyro_rate_filtered_data, 'b-', label='Filtered Gyro', linewidth=2)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Gyro Rate (deg/s)')
    axes[1].set_title('Gyroscope Z Rate - Raw vs Filtered')
    axes[1].legend()
    axes[1].grid(True)
    
    plt.tight_layout()
    plt.show()

def plot_controller_data():
    if len(time_data) == 0:
        print("No data to plot")
        return
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    axes[0].plot(time_data, fused_heading_data, 'b-', label='Fused Heading', linewidth=2)
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
    axes[2].plot(time_data, d_term_data, 'b-', label='D Term', linewidth=1)
    axes[2].plot(time_data, correction_data, 'k-', label='Control Signal', linewidth=2)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Correction (rad/s)')
    axes[2].set_title('Controller - PID Components and Control Signal')
    axes[2].legend()
    axes[2].grid(True)
    
    plt.tight_layout()
    plt.show()

try:
    while True:
        current_time = time.time()
        elapsed = current_time - start_time
        dt = current_time - last_time
        last_time = current_time
        
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            if line and ',' in line:
                values = line.split(',')
                if len(values) == 7:
                    data_count += 1
    
                    x = float(values[0])
                    y = float(values[1])
                    theta = float(values[2])
                    vl = float(values[3])
                    vr = float(values[4])
                    mpu_angle_deg = float(values[5])
                    gyro_rate_deg = float(values[6])
                    
                    gyro_rate_filtered = filter_gyro(gyro_rate_deg)
                    
                    mpu_angle_rad = mpu_angle_deg * math.pi / 180.0
                    
                    current_fused = fuse_theta(theta, gyro_rate_filtered, dt)
                    
                    correction, heading_error, p_term, i_term, d_term = compute_correction(current_fused, dt)
                    
                    if (current_time - last_command_time) >= command_interval:
                        command = f"V{linear_velocity},{correction}\n"
                        ser.write(command.encode())
                        last_command_time = current_time
                    
                    time_data.append(elapsed)
                    theta_data.append(theta)
                    mpu_angle_data.append(mpu_angle_rad)
                    gyro_rate_data.append(gyro_rate_deg)
                    gyro_rate_filtered_data.append(gyro_rate_filtered)
                    fused_heading_data.append(current_fused)
                    correction_data.append(correction)
                    p_term_data.append(p_term)
                    i_term_data.append(i_term)
                    d_term_data.append(d_term)
                    heading_error_data.append(heading_error)
                    
                    print(f"t={elapsed:.2f}s, "
                          f"theta={theta:.3f}, "
                          f"fused={current_fused:.3f}, "
                          f"error={heading_error:.3f}, "
                          f"corr={correction:.3f}")
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n\nData collection interrupted by user.")
    print(f"Total data points collected: {data_count}")
    print(f"Total duration: {time.time() - start_time:.2f} seconds")
    
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