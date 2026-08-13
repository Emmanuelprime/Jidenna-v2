import serial
import time

# Configure serial connection (adjust port as needed)
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Change this to your port (e.g., '/dev/ttyUSB0' on Linux)
    baudrate=115200,
    timeout=1
)

# Give the Arduino time to initialize
time.sleep(2)

# Flush any existing data
ser.reset_input_buffer()

# Test parameters
linear_velocity = 0.2    # m/s
angular_velocity = 0.1   # rad/s
duration = 5.0           # seconds

print("Starting data collection...")
print("Format: x, y, theta, vl, vr, mpu_z")

start_time = time.time()
command_interval = 0.1  # Send command every 100ms
last_command_time = 0

try:
    while (time.time() - start_time) < duration:
        current_time = time.time()
        
        # Send command periodically to prevent timeout
        if (current_time - last_command_time) >= command_interval:
            command = f"V{linear_velocity},{angular_velocity}\n"
            ser.write(command.encode())
            last_command_time = current_time
        
        # Read and print incoming data
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            if line and ',' in line:  # Validate data line
                print(f"Data: {line}")
                # Parse and use data if needed
                # values = line.split(',')
                # x = float(values[0])
                # y = float(values[1])
                # theta = float(values[2])
                # vl = float(values[3])
                # vr = float(values[4])
                # mpu_z = float(values[5])
        
        time.sleep(0.01)  # Small delay to prevent CPU overload

finally:
    # Stop the robot
    ser.write(b"V0,0\n")
    time.sleep(0.1)
    ser.close()
    print("\nData collection complete. Robot stopped.")