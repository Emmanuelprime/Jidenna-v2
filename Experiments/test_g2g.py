import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
from jidenna.heading_controller import HeadingHoldController
from jidenna.local_planner.g2g_controller import G2GController
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

heading_controller = HeadingHoldController()
print("Waiting for stabilization...")
time.sleep(2)
# No need for TurningController - G2G has internal turn control
g2g = G2GController(robot, heading_controller)

try:
    print(f"Going to goal: ({goal_x:.2f}, {goal_y:.2f}) at {speed:.2f} m/s")
    g2g.go_to_goal(goal_x, goal_y, speed=speed)
    
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    robot.stop()
    robot.disconnect()
    print("Done")