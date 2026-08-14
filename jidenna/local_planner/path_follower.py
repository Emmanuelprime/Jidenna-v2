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
        
        # Position tracking
        self.initial_pose = None
        self.current_pose = None
        
    def set_waypoints(self, waypoints):
        """Set waypoints relative to current position"""
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        
        # Store initial pose for relative calculations
        self.initial_pose = self.get_current_pose(timeout=5)
        if self.initial_pose:
            print(f"Initial pose: ({self.initial_pose[0]:.2f}, {self.initial_pose[1]:.2f})")
        
    def get_current_pose(self, timeout=3):
        """Get current robot pose"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            data = self.robot.serial.get_latest_data()
            if data:
                return data['x'], data['y'], data['theta']
            time.sleep(0.01)
        return None
    
    def get_heading(self, timeout=3):
        """Get current fused heading"""
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
    
    def shortest_angle_error(self, target, current):
        error = target - current
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        return error
    
    def turn_in_place(self, target_heading, timeout=8):
        """Turn in place to target heading"""
        print(f"  Turning to {target_heading:.2f} rad...")
        
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
            
            w = self.turning_controller.compute(current_heading, gyro_rate_rad, dt)
            
            if self.turning_controller.is_turn_complete(current_heading):
                print(f"  Turn complete!")
                break
            
            self.robot.drive(0, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
    
    def move_straight_to_waypoint(self, waypoint, speed=0.2):
        """Move straight to a waypoint using odometry"""
        # Get current position
        pose = self.get_current_pose(timeout=2)
        if pose is None:
            print("  ERROR: Cannot get position!")
            return False
        
        start_x, start_y, _ = pose
        
        # Calculate target distance and heading
        target_x, target_y = waypoint
        target_distance = math.sqrt((target_x - start_x)**2 + (target_y - start_y)**2)
        target_heading = math.atan2(target_y - start_y, target_x - start_x)
        
        print(f"  Moving {target_distance:.2f}m to waypoint...")
        
        # Turn to face waypoint
        current_heading = self.get_heading(timeout=2)
        if current_heading is not None:
            heading_error = self.shortest_angle_error(target_heading, current_heading)
            if abs(heading_error) > self.heading_tolerance:
                self.turn_in_place(target_heading)
        
        # Move straight
        self.heading_controller.set_target(target_heading)
        move_start_time = time.time()
        moved_distance = 0
        
        while moved_distance < target_distance - self.waypoint_tolerance:
            if time.time() - move_start_time > 15:
                print("  Move timeout!")
                break
            
            pose = self.get_current_pose(timeout=1)
            if pose is None:
                time.sleep(0.01)
                continue
            
            current_x, current_y, _ = pose
            moved_distance = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            
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
        time.sleep(0.3)
        print(f"  Waypoint reached!")
        return True
    
    def follow_path(self, speed=0.2):
        """Follow waypoints using relative movements"""
        if not self.waypoints:
            print("No waypoints set!")
            return
        
        print(f"Following path with {len(self.waypoints)} waypoints...")
        
        # Wait for initial data
        pose = self.get_current_pose(timeout=10)
        if pose is None:
            print("ERROR: Cannot get robot position!")
            return
        
        print(f"Starting position: ({pose[0]:.2f}, {pose[1]:.2f})")
        
        # Navigate to each waypoint relative to current position
        for i, waypoint in enumerate(self.waypoints):
            print(f"\n--- Waypoint {i+1}/{len(self.waypoints)} ---")
            
            # Skip first waypoint if it's the starting position
            if i == 0 and abs(waypoint[0]) < 0.01 and abs(waypoint[1]) < 0.01:
                print("  Skipping start waypoint")
                continue
            
            if not self.move_straight_to_waypoint(waypoint, speed):
                print("  Failed to reach waypoint!")
                break
        
        print("\nPath complete!")
        self.robot.stop()
    
    def stop(self):
        self.robot.stop()