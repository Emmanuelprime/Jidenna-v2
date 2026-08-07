import serial
import time
import csv

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)

csv_file = open('motor_data.csv', 'a', newline='')
writer = csv.writer(csv_file)

if csv_file.tell() == 0:
    writer.writerow(['x', 'y', 'theta', 'vl', 'vr', 'left_pwm', 'right_pwm'])

print("Interactive PWM Controller")
print("Commands:")
print("  L,R - Set left and right PWM (e.g., 30,40)")
print("  S - Stop motors")
print("  Q - Quit and save data")
print("  P - Print current data")
print()

while True:
    cmd = input("Enter PWM (L,R) or command: ").strip()
    
    if cmd.lower() == 'q':
        break
    
    elif cmd.lower() == 's':
        ser.write(b"S\n")
        print("Motors stopped")
        continue
    
    elif cmd.lower() == 'p':
        print(f"Data collected: {csv_file.tell() - 1} lines")
        continue
    
    elif ',' in cmd:
        parts = cmd.split(',')
        if len(parts) == 2:
            try:
                left_pwm = int(parts[0].strip())
                right_pwm = int(parts[1].strip())
                
                if -80 <= left_pwm <= 80 and -80 <= right_pwm <= 80:
                    cmd_str = f"M{left_pwm},{right_pwm}\n"
                    ser.write(cmd_str.encode())
                    print(f"Sending: {cmd_str.strip()}")
                    
                    start = time.time()
                    while time.time() - start < 3:
                        if ser.in_waiting:
                            line = ser.readline().decode().strip()
                            if line:
                                data = line.split(',')
                                if len(data) == 5:
                                    writer.writerow(data + [left_pwm, right_pwm])
                                    csv_file.flush()
                                    print(line)
                else:
                    print("PWM values must be between -80 and 80")
            except ValueError:
                print("Invalid PWM values. Use format: L,R (e.g., 30,40)")
        else:
            print("Invalid format. Use: L,R")
    else:
        print("Unknown command. Use L,R or S or Q")

csv_file.close()
ser.close()
print("Data saved to motor_data.csv")