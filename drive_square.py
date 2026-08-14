import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
import time

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()

try:
    print("Testing turn...")
    
    # Test if robot turns with positive angular velocity
    print("Turning left (positive w)...")
    robot.drive(0, 0.5)
    time.sleep(3)
    robot.stop()
    time.sleep(1)
    
    # Test if robot turns with negative angular velocity
    print("Turning right (negative w)...")
    robot.drive(0, -0.5)
    time.sleep(3)
    robot.stop()
    
    print("Test complete. Did the robot turn?")
    
except KeyboardInterrupt:
    print("Interrupted")
finally:
    robot.stop()
    robot.disconnect()