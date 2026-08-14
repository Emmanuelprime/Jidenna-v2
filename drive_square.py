import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
from jidenna.heading_controller import HeadingController
import time
import math

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()
heading_controller = HeadingController(kp=3.5, ki=0.6, kd=0.15, max_correction=0.8)

def get_current_heading(robot):
    """Get current fused heading"""
    data = robot.serial.get_latest_data()
    if data:
        gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
        fused = robot.fusion.update(data['theta'], gyro_rate_rad, data['timestamp'])
        if fused is not None:
            return fused
    return robot.get_heading()

def turn_to_heading(robot, heading_controller, target_heading, timeout=8):
    """Turn robot to a specific heading using heading controller"""
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
        
        error = heading_controller.target_heading - current_heading
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        print(f"  Current: {current_heading:.3f}, Error: {error:.3f}, w: {w:.3f}")
        
        if abs(error) < 0.05:
            print(f"  Turn complete! Error: {error:.3f} rad")
            break
        
        robot.drive(0, w)
        time.sleep(0.01)
    
    robot.stop()
    time.sleep(0.5)

def drive_straight_with_heading(robot, heading_controller, target_heading, duration=2, speed=0.2):
    """Drive straight maintaining target heading"""
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
    
    # Wait for initial heading
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
        target_heading = initial_heading + side * (math.pi / 2)
        while target_heading > math.pi:
            target_heading -= 2 * math.pi
        while target_heading < -math.pi:
            target_heading += 2 * math.pi
        
        # Drive straight maintaining heading
        drive_straight_with_heading(robot, heading_controller, target_heading, duration=2)
        
        # Turn to next heading (except after last side)
        if side < 3:
            next_heading = target_heading + math.pi / 2
            while next_heading > math.pi:
                next_heading -= 2 * math.pi
            while next_heading < -math.pi:
                next_heading += 2 * math.pi
            
            turn_to_heading(robot, heading_controller, next_heading)
    
    print("\n" + "="*60)
    print("Square complete with heading correction!")
    
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    robot.stop()
    robot.disconnect()
    print("Robot stopped and disconnected")