import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
from jidenna.heading_controller import HeadingController
import time
import math

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()
heading_controller = HeadingController()

def turn_to_heading(robot, heading_controller, target_heading, direction=1):
    """Turn robot to a specific heading using heading controller"""
    print(f"Turning to heading: {target_heading:.3f} rad ({target_heading*180/math.pi:.1f} deg)")
    
    # Set target heading
    heading_controller.set_target(target_heading)
    
    # Wait for heading data
    start_time = time.time()
    timeout = 8  # 8 second timeout
    
    while True:
        # Check timeout
        if time.time() - start_time > timeout:
            print("Turn timeout!")
            break
        
        current_heading = robot.get_heading()
        if current_heading is None:
            time.sleep(0.01)
            continue
        
        # Get gyro rate for D term
        data = robot.serial.get_latest_data()
        if data:
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
        else:
            gyro_rate_rad = 0.0
        
        # Compute correction (this gives us angular velocity)
        w = heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
        
        # Check if we've reached target (small error)
        error = heading_controller.target_heading - current_heading
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        if abs(error) < 0.05:  # Within ~3 degrees
            print(f"Turn complete! Error: {error:.3f} rad")
            break
        
        # Turn in place using the computed correction
        robot.drive(0, w)
        time.sleep(0.01)
    
    robot.stop()
    time.sleep(0.5)

def drive_straight_with_heading(robot, heading_controller, target_heading, duration=2):
    """Drive straight maintaining a specific heading"""
    print(f"Driving straight at heading: {target_heading:.3f} rad for {duration}s")
    
    heading_controller.set_target(target_heading)
    start_time = time.time()
    
    while time.time() - start_time < duration:
        current_heading = robot.get_heading()
        if current_heading is None:
            time.sleep(0.01)
            continue
        
        data = robot.serial.get_latest_data()
        if data:
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
        else:
            gyro_rate_rad = 0.0
        
        w = heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
        robot.drive(0.2, w)
        time.sleep(0.01)
    
    robot.stop()
    time.sleep(0.5)

try:
    print("Driving in a square using heading controller...")
    
    # Get initial heading
    initial_heading = None
    while initial_heading is None:
        initial_heading = robot.get_heading()
        time.sleep(0.01)
    
    print(f"\nInitial heading: {initial_heading:.3f} rad ({initial_heading*180/math.pi:.1f} deg)")
    
    # Square: 4 sides
    for i in range(4):
        print(f"\n--- Side {i+1} of 4 ---")
        
        # Calculate target heading for this side
        target_heading = initial_heading + i * (math.pi / 2)  # Add 90 degrees each side
        
        # Normalize target heading
        while target_heading > math.pi:
            target_heading -= 2 * math.pi
        while target_heading < -math.pi:
            target_heading += 2 * math.pi
        
        # Drive straight maintaining target heading
        drive_straight_with_heading(robot, heading_controller, target_heading, duration=2)
        
        # For the last side, don't turn
        if i < 3:
            # Calculate next target heading (turn 90 degrees)
            next_heading = target_heading + math.pi / 2
            while next_heading > math.pi:
                next_heading -= 2 * math.pi
            while next_heading < -math.pi:
                next_heading += 2 * math.pi
            
            # Turn to next heading
            turn_to_heading(robot, heading_controller, next_heading)
    
    print("\nSquare complete!")
    robot.stop()
    
except KeyboardInterrupt:
    print("\nInterrupted by user")
finally:
    robot.stop()
    robot.disconnect()
    print("Robot stopped and disconnected")