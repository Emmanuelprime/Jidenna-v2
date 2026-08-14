import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
import time
import math

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()

def turn_90_left(robot, turn_speed=0.5):
    """Turn 90 degrees left"""
    print("Turning 90 degrees LEFT...")
    turn_time = (math.pi / 2) / turn_speed  # ~3.14 seconds
    robot.drive(0, turn_speed)
    time.sleep(turn_time)
    robot.stop()
    time.sleep(0.5)

def move_forward(robot, duration=2, speed=0.2):
    """Move forward"""
    print(f"Moving forward for {duration}s...")
    start_time = time.time()
    
    while time.time() - start_time < duration:
        robot.drive(speed, 0)
        time.sleep(0.05)
    
    robot.stop()
    time.sleep(0.5)

try:
    print("Driving in a square...")
    
    for side in range(4):
        print(f"\n--- Side {side+1} of 4 ---")
        
        # Move forward 2 seconds
        move_forward(robot, duration=2)
        
        # Turn left 90 degrees (except after last side)
        if side < 3:
            turn_90_left(robot)
    
    print("\nSquare complete!")
    robot.stop()
    
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    robot.stop()
    robot.disconnect()
    print("Robot stopped and disconnected")