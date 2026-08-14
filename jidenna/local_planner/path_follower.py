import math
import time
from .turning_controller import TurningController

class PathFollower:
    def __init__(self, robot, heading_controller, turning_controller=None):
        self.robot = robot
        self.heading_controller = heading_controller
        self.turning_controller = turning_controller or TurningController()
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 0.15
        self.heading_tolerance = 0.1
        
    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        
    def get_current_pose(self, timeout=3):
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            data = self.robot.serial.get_latest_data()
            if data:
                return data['x'], data['y'], data['theta']
            time.sleep(0.01)
        
        return None
    
    def get_heading(self, timeout=3):
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            data = self.robot.serial.get_latest_data()
            if data:
                gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
                fused = self.robot.fusion.update(data['theta'], gyro_rate_rad, data['timestamp'])
                if fused is not None:
                    return fused
            time.sleep(0.01)
        
        return None
    
    def distance_to_waypoint(self, waypoint):
        pose = self.get_current_pose(timeout=1)
        if pose is None:
            return float('inf')
        x, y, _ = pose
        wx, wy = waypoint
        return math.sqrt((wx - x)**2 + (wy - y)**2)
    
    def angle_to_waypoint(self, waypoint):
        pose = self.get_current_pose(timeout=1)
        if pose is None:
            return None
        x, y, _ = pose
        wx, wy = waypoint
        return math.atan2(wy - y, wx - x)
    
    def shortest_angle_error(self, target, current):
        error = target - current
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        return error
    
    def turn_in_place(self, target_heading, timeout=8):
        print(f"  Turning in place to {target_heading:.2f} rad ({target_heading*180/math.pi:.1f} deg)...")
        
        self.turning_controller.set_target(target_heading)
        start_time = time.time()
        last_time = time.time()
        
        while True:
            if time.time() - start_time > timeout:
                print("  Turn timeout!")
                break
            
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            current_heading = self.get_heading(timeout=1)
            if current_heading is None:
                time.sleep(0.01)
                continue
            
            data = self.robot.serial.get_latest_data()
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
            
            # Compute turn velocity
            w = self.turning_controller.compute(current_heading, gyro_rate_rad, dt)
            
            error = self.shortest_angle_error(target_heading, current_heading)
            
            # Print progress every ~0.5 seconds
            if int(time.time() * 2) % 2 == 0:
                print(f"    Error: {error:.3f} rad ({error*180/math.pi:.1f} deg), w: {w:.3f}")
            
            # Check if turn is complete
            if self.turning_controller.is_turn_complete(current_heading):
                print(f"  Turn complete! Final error: {error:.3f} rad ({error*180/math.pi:.1f} deg)")
                break
            
            # Turn in place
            self.robot.drive(0, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
    
    def navigate_to_waypoint(self, waypoint, speed=0.2):
        print(f"\nNavigating to waypoint: ({waypoint[0]:.2f}, {waypoint[1]:.2f})")
        
        pose = self.get_current_pose(timeout=5)
        if pose is None:
            print("  ERROR: No position data available!")
            return
        
        desired_heading = self.angle_to_waypoint(waypoint)
        if desired_heading is None:
            print("  ERROR: Cannot calculate heading!")
            return
        
        # Phase 1: Turn in place
        current_heading = self.get_heading(timeout=5)
        if current_heading is not None:
            heading_error = self.shortest_angle_error(desired_heading, current_heading)
            if abs(heading_error) > self.heading_tolerance:
                self.turn_in_place(desired_heading)
        
        # Phase 2: Move straight
        print(f"  Moving straight to waypoint...")
        self.heading_controller.set_target(desired_heading)
        
        move_start_time = time.time()
        move_timeout = 15
        
        while self.distance_to_waypoint(waypoint) > self.waypoint_tolerance:
            if time.time() - move_start_time > move_timeout:
                print("  Move timeout!")
                break
            
            current_heading = self.get_heading(timeout=1)
            if current_heading is None:
                time.sleep(0.01)
                continue
            
            data = self.robot.serial.get_latest_data()
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
            
            w = self.heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
            
            self.robot.drive(speed, w)
            time.sleep(0.01)
        
        self.robot.stop()
        print(f"  Reached waypoint!")
        time.sleep(0.3)
    
    def follow_path(self, speed=0.2):
        if not self.waypoints:
            print("No waypoints set!")
            return
        
        print(f"Following path with {len(self.waypoints)} waypoints...")
        
        print("Waiting for robot data...")
        pose = self.get_current_pose(timeout=10)
        if pose is None:
            print("ERROR: Cannot get robot position!")
            return
        
        print(f"Initial position: ({pose[0]:.2f}, {pose[1]:.2f})")
        
        for i, waypoint in enumerate(self.waypoints):
            print(f"\n--- Waypoint {i+1}/{len(self.waypoints)} ---")
            self.navigate_to_waypoint(waypoint, speed)
        
        print("\nPath complete!")
        self.robot.stop()
    
    def stop(self):
        self.robot.stop()