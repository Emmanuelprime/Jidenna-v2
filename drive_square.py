import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
from jidenna.heading_controller import HeadingController
import time
import math

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()
heading_controller = HeadingController()

def get_current_heading(robot, timeout=5):
    """Get current heading with timeout"""
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        # Try to get heading
        heading = robot.get_heading()
        if heading is not None:
            return heading
        
        # Also try getting from fusion directly
        if robot.fusion and robot.fusion.fused_heading is not None:
            return robot.fusion.fused_heading
        
        # Try getting from latest data
        data = robot.serial.get_latest_data()
        if data:
            # Update fusion if needed
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
            fused = robot.fusion.update(data['theta'], gyro_rate_rad, data['timestamp'])
            if fused is not None:
                return fused
        
        time.sleep(0.01)
    
    print("Timeout waiting for heading!")
    return None

def turn_to_heading(robot, heading_controller, target_heading):
    """Turn robot to a specific heading"""
    print(f"Turning to heading: {target_heading:.3f} rad ({target_heading*180/math.pi:.1f} deg)")
    
    heading_controller.set_target(target_heading)
    start_time = time.time()
    timeout = 8
    
    while True:
        if time.time() - start_time > timeout:
            print("Turn timeout!")
            break
        
        current_heading = get_current_heading(robot, timeout=1)
        if current_heading is None:
            print("No heading data, trying to continue...")
            robot.drive(0, 0.3)  # Slow turn as fallback
            time.sleep(0.1)
            continue
        
        # Get gyro rate
        data = robot.serial.get_latest_data()
        gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
        
        # Compute correction
        w = heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
        
        # Check error
        error = heading_controller.target_heading - current_heading
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        print(f"Current: {current_heading:.3f}, Target: {target_heading:.3f}, Error: {error:.3f}, w: {w:.3f}")
        
        if abs(error) < 0.05:
            print("Turn complete!")
            break
        
        robot.drive(0, w)
        time.sleep(0.01)
    
    robot.stop()
    time.sleep(0.5)

def drive_straight_with_heading(robot, heading_controller, target_heading, duration=2):
    """Drive straight maintaining heading"""
    print(f"Driving straight at {target_heading:.3f} rad for {duration}s")
    
    heading_controller.set_target(target_heading)
    start_time = time.time()
    
    while time.time() - start_time < duration:
        current_heading = get_current_heading(robot, timeout=0.5)
        if current_heading is None:
            robot.drive(0.2, 0)  # Just go straight if no heading
            time.sleep(0.01)
            continue
        
        data = robot.serial.get_latest_data()
        gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
        
        w = heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
        robot.drive(0.2, w)
        time.sleep(0.01)
    
    robot.stop()
    time.sleep(0.5)

try:
    print("Driving in a square...")
    
    # Wait for initial heading with debug
    print("Waiting for initial heading...")
    initial_heading = get_current_heading(robot, timeout=10)
    
    if initial_heading is None:
        print("ERROR: Could not get initial heading!")
        print("Check if robot is sending data")
        sys.exit(1)
    
    print(f"Initial heading: {initial_heading:.3f} rad ({initial_heading*180/math.pi:.1f} deg)")
    
    for i in range(4):
        print(f"\n--- Side {i+1} of 4 ---")
        
        # Calculate target heading
        target_heading = initial_heading + i * (math.pi / 2)
        while target_heading > math.pi:
            target_heading -= 2 * math.pi
        while target_heading < -math.pi:
            target_heading += 2 * math.pi
        
        # Drive straight
        drive_straight_with_heading(robot, heading_controller, target_heading, duration=2)
        
        # Turn (except after last side)
        if i < 3:
            next_heading = target_heading + math.pi / 2
            while next_heading > math.pi:
                next_heading -= 2 * math.pi
            while next_heading < -math.pi:
                next_heading += 2 * math.pi
            
            turn_to_heading(robot, heading_controller, next_heading)
    
    print("\nSquare complete!")
    
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    robot.stop()
    robot.disconnect()