import math
import time

class PurePursuit:
    def __init__(self, robot, heading_controller, lookahead_distance=0.4):
        self.robot = robot
        self.heading_controller = heading_controller
        self.lookahead_distance = lookahead_distance
        self.path = []
        
    def set_path(self, path):
        """Set path as list of (x, y) points"""
        self.path = path
        
    def get_pose(self):
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
    
    def find_lookahead_point(self, current_pos):
        """Find point on path at lookahead distance"""
        if not self.path:
            return None
        
        cx, cy = current_pos
        
        # Find closest point on path
        closest_point = None
        closest_dist = float('inf')
        
        for point in self.path:
            dist = math.sqrt((point[0] - cx)**2 + (point[1] - cy)**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_point = point
        
        if closest_point is None:
            return None
        
        # Find point at lookahead distance
        for point in self.path:
            dist = math.sqrt((point[0] - cx)**2 + (point[1] - cy)**2)
            if dist >= self.lookahead_distance:
                return point
        
        return self.path[-1]
    
    def follow_path(self, speed=0.2, timeout=30):
        """Follow path using pure pursuit"""
        print("Following path with pure pursuit...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            pose = self.get_pose()
            if pose is None:
                time.sleep(0.01)
                continue
            
            x, y, _ = pose
            lookahead = self.find_lookahead_point((x, y))
            
            if lookahead is None:
                break
            
            # Calculate heading to lookahead point
            desired_heading = math.atan2(lookahead[1] - y, lookahead[0] - x)
            current_heading = self.get_heading()
            
            if current_heading is None:
                time.sleep(0.01)
                continue
            
            self.heading_controller.set_target(desired_heading)
            
            data = self.robot.serial.get_latest_data()
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
            
            w = self.heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
            
            self.robot.drive(speed, w)
            
            # Check if reached end of path
            if len(self.path) > 0:
                end_point = self.path[-1]
                dist_to_end = math.sqrt((end_point[0] - x)**2 + (end_point[1] - y)**2)
                if dist_to_end < 0.15:
                    print("Reached end of path")
                    break
            
            time.sleep(0.01)
        
        self.robot.stop()