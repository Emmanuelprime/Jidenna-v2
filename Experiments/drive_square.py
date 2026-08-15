import sys
sys.path.append('..')
from Experiments.jidenna.jidenna import Jidenna
from Experiments.jidenna.heading_controller import HeadingController
import time
import math

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()
heading_controller = HeadingController(kp=3.5, ki=0.6, kd=0.15, max_correction=0.8)

def get_current_heading(robot):
    data = robot.serial.get_latest_data()
    if data:
        gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
        fused = robot.fusion.update(data['theta'], gyro_rate_rad, data['timestamp'])
        if fused is not None:
            return fused
    return robot.get_heading()

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def shortest_angle_error(target, current):
    error = target - current
    while error > math.pi:
        error -= 2 * math.pi
    while error < -math.pi:
        error += 2 * math.pi
    return error

def turn_to_heading(robot, heading_controller, target_heading, timeout=8):
    print(f"Turning to heading: {target_heading:.3f} rad ({target_heading*180/math.pi:.1f} deg)")
    
    heading_controller.set_target(target_heading)
    start_time = time.time()
    
    while True:
        if time.time() - start_time > timeout:
            print("Turn timeout!")
            break
        
        current_heading = get_current_heading(robot)
        if current_heading is None:
            time.sleep(0.01)
            continue
        
        data = robot.serial.get_latest_data()
        gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
        
        w = heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
        
        error = shortest_angle_error(target_heading, current_heading)
        
        if abs(error) < 0.05:
            print(f"  Turn complete! Error: {error:.3f} rad")
            break
        
        robot.drive(0, w)
        time.sleep(0.01)
    
    robot.stop()
    time.sleep(0.5)

def drive_straight_with_heading(robot, heading_controller, target_heading, duration=2, speed=0.2):
    print(f"Driving straight at {target_heading:.3f} rad for {duration}s...")
    
    heading_controller.set_target(target_heading)
    start_time = time.time()
    
    while time.time() - start_time < duration:
        current_heading = get_current_heading(robot)
        if current_heading is None:
            time.sleep(0.01)
            continue
        
        data = robot.serial.get_latest_data()
        gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
        
        w = heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
        robot.drive(speed, w)
        time.sleep(0.01)
    
    robot.stop()
    time.sleep(0.5)

try:
    print("Driving in a square with heading correction...")
    
    print("Waiting for initial heading...")
    initial_heading = None
    start_wait = time.time()
    while initial_heading is None and time.time() - start_wait < 10:
        initial_heading = get_current_heading(robot)
        time.sleep(0.01)
    
    if initial_heading is None:
        print("ERROR: Could not get heading!")
        sys.exit(1)
    
    print(f"Initial heading: {initial_heading:.3f} rad ({initial_heading*180/math.pi:.1f} deg)")
    print("-"*60)
    
    for side in range(4):
        print(f"\n--- Side {side+1} of 4 ---")
        
        # Calculate target heading for this side
        target_heading = normalize_angle(initial_heading + side * (math.pi / 2))
        
        # Drive straight maintaining heading
        drive_straight_with_heading(robot, heading_controller, target_heading, duration=2)
        
        # Turn to next heading (except after last side)
        if side < 3:
            next_heading = normalize_angle(target_heading + math.pi / 2)
            turn_to_heading(robot, heading_controller, next_heading)
    
    print("\n" + "="*60)
    print("Square complete with heading correction!")
    
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    robot.stop()
    robot.disconnect()
    print("Robot stopped and disconnected")