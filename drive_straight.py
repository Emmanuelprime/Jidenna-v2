import sys
from jidenna import Jidenna

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
speed = float(sys.argv[3]) if len(sys.argv) > 3 else 0.2

robot = Jidenna(port)
robot.connect()

print(f"Driving straight for {duration}s at {speed} m/s...")
robot.drive_straight(speed, duration)

robot.disconnect()
print("Done")