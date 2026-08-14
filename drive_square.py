import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
import time
import math

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()

def turn_90_degrees(robot, direction=1):
    """Turn robot 90 degrees using time-based approach"""
    # At 0.5 rad/s, 90 degrees (pi/2 rad) takes pi seconds
    turn_time = math.pi / 0.5  # ~3.14 seconds for 90 degrees
    
    print(f"Turning {direction * 90} degrees...")
    robot.drive(0, direction * 0.5)
    time.sleep(turn_time)
    robot.stop()
    time.sleep(1)  # Let robot settle

def move_forward(robot, duration=2):
    """Move forward with heading correction"""
    print(f"Moving forward for {duration} seconds...")
    robot.drive_straight(0.2, duration=duration)
    time.sleep(0.5)  # Brief pause

try:
    print("Driving in a square...")
    
    for i in range(4):
        print(f"\n--- Side {i+1} of 4 ---")
        
        # Move forward 2 seconds
        move_forward(robot, duration=2)
        
        # Turn 90 degrees left
        turn_90_degrees(robot, direction=1)
    
    print("\nSquare complete!")
    robot.stop()
    
except KeyboardInterrupt:
    print("\nInterrupted by user")
finally:
    robot.stop()
    robot.disconnect()
    print("Robot stopped and disconnected")