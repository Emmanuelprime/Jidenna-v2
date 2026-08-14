import sys
sys.path.append('..')
from jidenna.jidenna import Jidenna
from jidenna.heading_controller import HeadingController
from jidenna.local_planner.path_follower import PathFollower
from jidenna.local_planner.turning_controller import TurningController
from jidenna.local_planner.trajectory import TrajectoryGenerator
import time

robot = Jidenna(port='/dev/ttyUSB0')
robot.connect()
heading_controller = HeadingController()

turning_controller = TurningController(
    kp=2.5,
    ki=0.05,
    kd=0.4,
    max_angular_velocity=0.8
)

path_follower = PathFollower(robot, heading_controller, turning_controller)
trajectory_gen = TrajectoryGenerator()

# Generate square with RELATIVE waypoints
print("Generating square trajectory (relative)...")
square_path = trajectory_gen.generate_square(
    side_length=1.0,
    num_points_per_side=2,
    relative=True  # Use relative movements
)

# Extract unique waypoints
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