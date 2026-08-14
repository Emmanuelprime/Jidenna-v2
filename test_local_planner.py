import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
from jidenna.heading_controller import HeadingController
from jidenna.local_planner import PathFollower, TrajectoryGenerator
import time

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()
heading_controller = HeadingController()

path_follower = PathFollower(robot, heading_controller)
trajectory_gen = TrajectoryGenerator()

# Generate square path
square_path = trajectory_gen.generate_square(1.0)  # 1m square

# Set waypoints (corners of square)
waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
path_follower.set_waypoints(waypoints)

# Follow the path
try:
    path_follower.follow_path(speed=0.2)
except KeyboardInterrupt:
    print("Interrupted")
finally:
    robot.stop()
    robot.disconnect()