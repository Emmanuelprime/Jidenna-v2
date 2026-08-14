import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
from jidenna.heading_controller import HeadingController
from jidenna.local_planner.g2g_controller import G2GController
from jidenna.local_planner.turning_controller import TurningController
import time

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()
heading_controller = HeadingController()
turning_controller = TurningController()

g2g = G2GController(robot, heading_controller, turning_controller)

try:
    # Test: Go forward 1m
    g2g.go_to_goal_with_phases(1.0, 0.0, speed=0.2)
    
    #Test: Go left 1m
    g2g.go_to_goal_with_phases(0.0, 1.0, speed=0.2)
    
    # Test: Go diagonal 1m
    g2g.go_to_goal_with_phases(0.7, 0.7, speed=0.2)
    
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    robot.stop()
    robot.disconnect()
    print("Done")