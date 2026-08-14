import math
import time

class PathFollower:
    def __init__(self, robot, heading_controller):
        self.robot = robot
        self.heading_controller = heading_controller
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 0.15  # 15cm from waypoint
        self.heading_tolerance = 0.1    # ~6 degrees
        
    def set_waypoints(self, waypoints):
        """Set list of waypoints [(x1,y1), (x2,y2), ...]"""
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        
    def get_current_pose(self):
        """Get current robot pose (x, y, theta)"""
        data = self.robot.serial.get_latest_data()
        if data:
            return data['x'], data['y'], data['theta']
        return None
    
    def get_heading(self):
        """Get current fused heading"""
        data = self.robot.serial.get_latest_data()
        if data:
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
            fused = self.robot.fusion.update(data['theta'], gyro_rate_rad, data['timestamp'])
            if fused is not None:
                return fused
        return None
    
    def distance_to_waypoint(self, waypoint):
        """Calculate distance from robot to waypoint"""
        pose = self.get_current_pose()
        if pose is None:
            return float('inf')
        
        x, y, _ = pose
        wx, wy = waypoint
        return math.sqrt((wx - x)**2 + (wy - y)**2)
    
    def angle_to_waypoint(self, waypoint):
        """Calculate angle from robot to waypoint"""
        pose = self.get_current_pose()
        if pose is None:
            return None
        
        x, y, _ = pose
        wx, wy = waypoint
        return math.atan2(wy - y, wx - x)
    
    def navigate_to_waypoint(self, waypoint, speed=0.2):
        """Navigate to a single waypoint"""
        print(f"Navigating to waypoint: ({waypoint[0]:.2f}, {waypoint[1]:.2f})")
        
        while self.distance_to_waypoint(waypoint) > self.waypoint_tolerance:
            # Get current position
            pose = self.get_current_pose()
            if pose is None:
                time.sleep(0.01)
                continue
            
            # Calculate desired heading to waypoint
            desired_heading = self.angle_to_waypoint(waypoint)
            current_heading = self.get_heading()
            
            if desired_heading is None or current_heading is None:
                time.sleep(0.01)
                continue
            
            # Set heading target
            self.heading_controller.set_target(desired_heading)
            
            # Get gyro rate for D term
            data = self.robot.serial.get_latest_data()
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
            
            # Compute angular correction
            w = self.heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
            
            # Send velocity command
            self.robot.drive(speed, w)
            time.sleep(0.01)
        
        self.robot.stop()
        print(f"Reached waypoint: ({waypoint[0]:.2f}, {waypoint[1]:.2f})")
        time.sleep(0.5)
    
    def follow_path(self, speed=0.2):
        """Follow all waypoints in sequence"""
        if not self.waypoints:
            print("No waypoints set!")
            return
        
        print(f"Following path with {len(self.waypoints)} waypoints...")
        
        for i, waypoint in enumerate(self.waypoints):
            print(f"\nWaypoint {i+1}/{len(self.waypoints)}")
            self.navigate_to_waypoint(waypoint, speed)
        
        print("\nPath complete!")
        self.robot.stop()
    
    def stop(self):
        self.robot.stop()