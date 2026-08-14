import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
import time
import math

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()

def turn_90_degrees(robot, direction=1):
    """Turn robot 90 degrees"""
    start_heading = robot.get_heading()
    target_change = direction * math.pi / 2
    
    while True:
        current_heading = robot.get_heading()
        if current_heading is None:
            continue
        
        # Calculate how much we've turned
        heading_change = current_heading - start_heading
        while heading_change > math.pi:
            heading_change -= 2 * math.pi
        while heading_change < -math.pi:
            heading_change += 2 * math.pi
        
        if abs(heading_change) >= abs(target_change):
            break
        
        # Turn in place
        robot.drive(0, direction * 0.5)
        time.sleep(0.01)
    
    robot.stop()

try:
    # Drive in a square
    for i in range(4):
        robot.drive_straight(0.2, duration=2)
        turn_90_degrees(robot, direction=1)
    
except KeyboardInterrupt:
    print("Interrupted")
finally:
    robot.disconnect()