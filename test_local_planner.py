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

# Generate square with just the corners (4 waypoints + start)
print("Generating square trajectory...")
square_path = trajectory_gen.generate_square(
    side_length=1.0,
    start=(0, 0),
    num_points_per_side=2  # Just corners
)

# Extract unique waypoints (remove duplicates)
waypoints = []
for point in square_path:
    if not waypoints or waypoints[-1] != point:
        waypoints.append(point)

print(f"Using {len(waypoints)} waypoints:")
for i, wp in enumerate(waypoints):
    print(f"  Waypoint {i+1}: ({wp[0]:.2f}, {wp[1]:.2f})")

path_follower.set_waypoints(waypoints)

try:
    path_follower.follow_path(speed=0.2)
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    robot.stop()
    robot.disconnect()
    print("Done")