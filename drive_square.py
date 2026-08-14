import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
import time
import math

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()

def turn_90_degrees(robot, direction=1, timeout=5):
    """Turn robot 90 degrees"""
    # Wait for heading to be available
    start_heading = None
    while start_heading is None:
        start_heading = robot.get_heading()
        time.sleep(0.01)
    
    print(f"Starting heading: {start_heading:.3f} rad")
    
    target_change = direction * math.pi / 2
    start_time = time.time()
    
    while True:
        # Check timeout
        if time.time() - start_time > timeout:
            print("Turn timeout - stopping")
            break
        
        current_heading = robot.get_heading()
        if current_heading is None:
            continue
        
        # Calculate how much we've turned
        heading_change = current_heading - start_heading
        # Normalize to [-pi, pi]
        while heading_change > math.pi:
            heading_change -= 2 * math.pi
        while heading_change < -math.pi:
            heading_change += 2 * math.pi
        
        print(f"Current: {current_heading:.3f}, Change: {heading_change:.3f}, Target: {target_change:.3f}")
        
        # Check if we've turned enough
        if abs(heading_change) >= abs(target_change):
            print("Turn complete")
            break
        
        # Turn in place
        robot.drive(0, direction * 0.5)
        time.sleep(0.01)
    
    robot.stop()
    time.sleep(0.5)

try:
    # Drive in a square
    for i in range(4):
        print(f"\n--- Side {i+1} ---")
        robot.drive_straight(0.2, duration=2)
        turn_90_degrees(robot, direction=1)
    
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    robot.disconnect()