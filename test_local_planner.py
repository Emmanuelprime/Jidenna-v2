import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
from jidenna.heading_controller import HeadingController
from jidenna.local_planner.path_follower import PathFollower
from jidenna.local_planner.trajectory import TrajectoryGenerator
import time

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()
heading_controller = HeadingController()

path_follower = PathFollower(robot, heading_controller)
trajectory_gen = TrajectoryGenerator()

# Generate square trajectory with more points for smoother path
print("Generating square trajectory...")
square_path = trajectory_gen.generate_square(
    side_length=1.0,
    start=(0, 0),
    num_points_per_side=10
)

print(f"Generated {len(square_path)} trajectory points")
print(f"First point: {square_path[0]}")
print(f"Last point: {square_path[-1]}")

# Use all trajectory points as waypoints
path_follower.set_waypoints(square_path)

try:
    path_follower.follow_path(speed=0.15)  # Slower for dense trajectory
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    robot.stop()
    robot.disconnect()
    print("Done")