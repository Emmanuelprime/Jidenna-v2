import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
from jidenna.heading_controller import HeadingController
from jidenna.local_planner.g2g_controller import G2GController
from jidenna.local_planner.turning_controller import TurningController
import time

if len(sys.argv) >= 3:
    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
else:
    goal_x = 1.0
    goal_y = 0.0
    print("Using default goal: (1.0, 0.0)")
    print("Usage: python test_g2g.py <goal_x> <goal_y> [speed]")

speed = 0.2
if len(sys.argv) >= 4:
    speed = float(sys.argv[3])

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()
heading_controller = HeadingController()
turning_controller = TurningController()

g2g = G2GController(robot, heading_controller, turning_controller)

try:
    print(f"Going to goal: ({goal_x:.2f}, {goal_y:.2f}) at {speed:.2f} m/s")
    
    # Use go_to_goal_with_phases for backward movement
    if goal_x < 0:
        print("Using phased navigation for backward goal...")
        g2g.go_to_goal_with_phases(goal_x, goal_y, speed=speed)
    else:
        g2g.go_to_goal(goal_x, goal_y, speed=speed)
    
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    robot.stop()
    robot.disconnect()
    print("Done")