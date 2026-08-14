import math
import time

class G2GController:
    def __init__(self, robot, heading_controller, turning_controller):
        self.robot = robot
        self.heading_controller = heading_controller
        self.turning_controller = turning_controller
        
        # Controller parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.distance_tolerance = 0.1
        self.heading_tolerance = 0.1
        self.timeout = 15
        
        # State
        self.is_navigating = False
        self.goal_reached = False
        
    def get_pose(self, timeout=3):
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
    
    def shortest_angle_error(self, target, current):
        error = target - current
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        return error
    
    def go_to_goal(self, goal_x, goal_y, speed=None):
        """Navigate to a goal position (x, y) relative to current position"""
        if speed:
            self.linear_speed = speed
        
        self.is_navigating = True
        self.goal_reached = False
        
        # Get current position
        pose = self.get_pose(timeout=5)
        if pose is None:
            print("ERROR: Cannot get position!")
            self.is_navigating = False
            return False
        
        start_x, start_y, _ = pose
        
        # Calculate target in absolute coordinates (current + goal offset)
        target_x = start_x + goal_x
        target_y = start_y + goal_y
        
        print(f"Going to goal: ({goal_x:.2f}, {goal_y:.2f})")
        print(f"Target position: ({target_x:.2f}, {target_y:.2f})")
        
        start_time = time.time()
        
        while self.is_navigating:
            # Check timeout
            if time.time() - start_time > self.timeout:
                print("Navigation timeout!")
                break
            
            # Get current position
            pose = self.get_pose(timeout=1)
            if pose is None:
                time.sleep(0.01)
                continue
            
            current_x, current_y, _ = pose
            
            # Calculate distance to goal
            distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            # Check if goal reached
            if distance < self.distance_tolerance:
                print(f"Goal reached! Distance: {distance:.3f}m")
                self.goal_reached = True
                break
            
            # Calculate desired heading to goal
            desired_heading = math.atan2(target_y - current_y, target_x - current_x)
            
            # Get current heading
            current_heading = self.get_heading(timeout=1)
            if current_heading is None:
                time.sleep(0.01)
                continue
            
            # Calculate heading error
            heading_error = self.shortest_angle_error(desired_heading, current_heading)
            
            # Decide: turn or move forward
            if abs(heading_error) > self.heading_tolerance:
                # Turn in place to face goal
                w = self.turning_controller.compute(current_heading, 0, 0.05)
                self.robot.drive(0, w)
            else:
                # Move forward with heading correction
                data = self.robot.serial.get_latest_data()
                gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
                
                self.heading_controller.set_target(desired_heading)
                w = self.heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
                
                # Scale speed based on distance (slow down when close)
                speed = self.linear_speed
                if distance < 0.3:
                    speed = self.linear_speed * (distance / 0.3)
                
                self.robot.drive(speed, w)
            
            time.sleep(0.01)
        
        # Stop robot
        self.robot.stop()
        time.sleep(0.3)
        self.is_navigating = False
        
        return self.goal_reached
    
    def go_to_goal_with_phases(self, goal_x, goal_y, speed=None):
        """Navigate to goal with distinct turn and move phases"""
        if speed:
            self.linear_speed = speed
        
        self.is_navigating = True
        self.goal_reached = False
        
        pose = self.get_pose(timeout=5)
        if pose is None:
            print("ERROR: Cannot get position!")
            self.is_navigating = False
            return False
        
        start_x, start_y, _ = pose
        target_x = start_x + goal_x
        target_y = start_y + goal_y
        
        print(f"Going to goal: ({goal_x:.2f}, {goal_y:.2f})")
        
        # Phase 1: Turn to face goal
        desired_heading = math.atan2(goal_y, goal_x)
        current_heading = self.get_heading(timeout=5)
        
        if current_heading is not None:
            heading_error = self.shortest_angle_error(desired_heading, current_heading)
            if abs(heading_error) > self.heading_tolerance:
                print(f"Turning to face goal...")
                self._turn_to_heading(desired_heading)
        
        # Phase 2: Move straight to goal
        print(f"Moving to goal...")
        distance = math.sqrt(goal_x**2 + goal_y**2)
        self._move_distance(distance, desired_heading, speed or self.linear_speed)
        
        self.robot.stop()
        time.sleep(0.3)
        self.is_navigating = False
        self.goal_reached = True
        
        return True
    
    def _turn_to_heading(self, target_heading, timeout=8):
        """Turn in place to target heading"""
        self.robot.stop()
        time.sleep(0.3)
        
        self.turning_controller.set_target(target_heading)
        start_time = time.time()
        last_time = time.time()
        
        while True:
            if time.time() - start_time > timeout:
                print("Turn timeout!")
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
                break
            
            self.robot.drive(0, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
    
    def _move_distance(self, distance, heading, speed=0.2):
        """Move straight for a specific distance"""
        self.heading_controller.set_target(heading)
        start_time = time.time()
        moved_distance = 0
        start_pose = self.get_pose(timeout=2)
        
        if start_pose is None:
            return
        
        start_x, start_y, _ = start_pose
        
        while moved_distance < distance - self.distance_tolerance:
            if time.time() - start_time > self.timeout:
                print("Move timeout!")
                break
            
            pose = self.get_pose(timeout=1)
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
            
            # Slow down when close
            remaining = distance - moved_distance
            if remaining < 0.3:
                speed = 0.2 * (remaining / 0.3)
            
            self.robot.drive(speed, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
    
    def stop(self):
        self.is_navigating = False
        self.robot.stop()