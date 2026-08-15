import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
import time

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()

try:
    robot.drive_straight(0.5, duration=5)
    
except KeyboardInterrupt:
    print("Interrupted")
finally:
    robot.disconnect()