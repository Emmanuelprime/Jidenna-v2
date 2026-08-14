import math
import time

class PathFollower:
    def __init__(self, robot, heading_controller):
        self.robot = robot
        self.heading_controller = heading_controller
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 0.15
        self.heading_tolerance = 0.1  # ~6 degrees
        
    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        
    def get_current_pose(self):
        data = self.robot.serial.get_latest_data()
        if data:
            return data['x'], data['y'], data['theta']
        return None
    
    def get_heading(self):
        data = self.robot.serial.get_latest_data()
        if data:
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
            fused = self.robot.fusion.update(data['theta'], gyro_rate_rad, data['timestamp'])
            if fused is not None:
                return fused
        return None
    
    def distance_to_waypoint(self, waypoint):
        pose = self.get_current_pose()
        if pose is None:
            return float('inf')
        x, y, _ = pose
        wx, wy = waypoint
        return math.sqrt((wx - x)**2 + (wy - y)**2)
    
    def angle_to_waypoint(self, waypoint):
        pose = self.get_current_pose()
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
        """Turn in place to target heading"""
        print(f"  Turning in place to {target_heading:.2f} rad...")
        
        self.heading_controller.set_target(target_heading)
        start_time = time.time()
        
        while True:
            if time.time() - start_time > timeout:
                print("  Turn timeout!")
                break
            
            current_heading = self.get_heading()
            if current_heading is None:
                time.sleep(0.01)
                continue
            
            data = self.robot.serial.get_latest_data()
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
            
            w = self.heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
            
            error = self.shortest_angle_error(target_heading, current_heading)
            
            if abs(error) < self.heading_tolerance:
                print(f"  Turn complete! Error: {error:.3f} rad")
                break
            
            # Turn in place (v=0)
            self.robot.drive(0, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
    
    def move_straight(self, target_heading, speed=0.2, timeout=10):
        """Move straight maintaining heading"""
        self.heading_controller.set_target(target_heading)
        start_time = time.time()
        
        while True:
            if time.time() - start_time > timeout:
                print("  Move timeout!")
                break
            
            current_heading = self.get_heading()
            if current_heading is None:
                time.sleep(0.01)
                continue
            
            data = self.robot.serial.get_latest_data()
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
            
            w = self.heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
            
            # Move forward with heading correction
            self.robot.drive(speed, w)
            time.sleep(0.01)
    
    def navigate_to_waypoint(self, waypoint, speed=0.2):
        """Navigate to waypoint: turn in place first, then move straight"""
        print(f"\nNavigating to waypoint: ({waypoint[0]:.2f}, {waypoint[1]:.2f})")
        
        # Calculate desired heading to waypoint
        desired_heading = self.angle_to_waypoint(waypoint)
        if desired_heading is None:
            print("  Cannot get position!")
            return
        
        # Phase 1: Turn in place to face waypoint
        self.turn_in_place(desired_heading)
        
        # Phase 2: Move straight until reaching waypoint
        print(f"  Moving straight to waypoint...")
        self.heading_controller.set_target(desired_heading)
        
        while self.distance_to_waypoint(waypoint) > self.waypoint_tolerance:
            current_heading = self.get_heading()
            if current_heading is None:
                time.sleep(0.01)
                continue
            
            data = self.robot.serial.get_latest_data()
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
            
            w = self.heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
            
            # Move forward with heading correction
            self.robot.drive(speed, w)
            time.sleep(0.01)
        
        self.robot.stop()
        print(f"  Reached waypoint!")
        time.sleep(0.5)
    
    def follow_path(self, speed=0.2):
        if not self.waypoints:
            print("No waypoints set!")
            return
        
        print(f"Following path with {len(self.waypoints)} waypoints...")
        
        for i, waypoint in enumerate(self.waypoints):
            print(f"\n--- Waypoint {i+1}/{len(self.waypoints)} ---")
            self.navigate_to_waypoint(waypoint, speed)
        
        print("\nPath complete!")
        self.robot.stop()
    
    def stop(self):
        self.robot.stop()