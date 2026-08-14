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
        self.distance_tolerance = 0.20
        self.heading_tolerance = 0.15
        self.timeout = 25
        
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
        
        pose = self.get_pose(timeout=5)
        if pose is None:
            print("ERROR: Cannot get position!")
            self.is_navigating = False
            return False
        
        start_x, start_y, _ = pose
        target_x = start_x + goal_x
        target_y = start_y + goal_y
        
        print(f"Going to goal: ({goal_x:.2f}, {goal_y:.2f})")
        
        start_time = time.time()
        last_print_time = 0
        
        while self.is_navigating:
            if time.time() - start_time > self.timeout:
                print(f"Navigation timeout! Time: {time.time() - start_time:.1f}s")
                break
            
            pose = self.get_pose(timeout=1)
            if pose is None:
                time.sleep(0.01)
                continue
            
            current_x, current_y, _ = pose
            distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            if distance < self.distance_tolerance:
                print(f"Goal reached! Distance: {distance:.3f}m")
                self.goal_reached = True
                break
            
            desired_heading = math.atan2(target_y - current_y, target_x - current_x)
            current_heading = self.get_heading(timeout=1)
            
            if current_heading is None:
                time.sleep(0.01)
                continue
            
            heading_error = self.shortest_angle_error(desired_heading, current_heading)
            
            if time.time() - last_print_time > 1:
                print(f"  Dist: {distance:.2f}m, Heading err: {heading_error:.2f} rad ({heading_error*180/math.pi:.0f} deg)")
                last_print_time = time.time()
            
            if abs(heading_error) > self.heading_tolerance:
                # Turn toward goal
                w = 2.0 * heading_error
                w = max(-0.8, min(0.8, w))
                self.robot.drive(0, w)
            else:
                # Move forward
                data = self.robot.serial.get_latest_data()
                gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
                
                self.heading_controller.set_target(desired_heading)
                w = self.heading_controller.compute(current_heading, gyro_rate_rad, 0.05)
                
                # Keep constant speed until very close
                move_speed = self.linear_speed
                if distance < 0.5:
                    move_speed = max(0.1, self.linear_speed * (distance / 0.5))
                
                self.robot.drive(move_speed, w)
            
            time.sleep(0.01)
        
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
        
        print(f"Going to goal: ({goal_x:.2f}, {goal_y:.2f})")
        
        # Phase 1: Turn to face goal
        desired_heading = math.atan2(goal_y, goal_x)
        current_heading = self.get_heading(timeout=5)
        
        if current_heading is not None:
            heading_error = self.shortest_angle_error(desired_heading, current_heading)
            if abs(heading_error) > self.heading_tolerance:
                print(f"Turning to face goal ({heading_error*180/math.pi:.0f} deg)...")
                self._turn_to_heading(desired_heading)
        
        # Phase 2: Move straight to goal
        distance = math.sqrt(goal_x**2 + goal_y**2)
        print(f"Moving {distance:.2f}m to goal...")
        
        self.robot.stop()
        time.sleep(0.3)
        
        self.heading_controller.set_target(desired_heading)
        start_time = time.time()
        start_pose = self.get_pose(timeout=2)
        
        if start_pose is None:
            self.is_navigating = False
            return False
        
        start_x, start_y, _ = start_pose
        moved_distance = 0
        
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
            
            # Slow down near goal but maintain minimum speed
            remaining = distance - moved_distance
            move_speed = self.linear_speed
            if remaining < 0.5:
                move_speed = max(0.1, self.linear_speed * (remaining / 0.5))
            
            self.robot.drive(move_speed, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
        self.is_navigating = False
        self.goal_reached = True
        print("Goal reached!")
        
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
            
            remaining = distance - moved_distance
            move_speed = speed
            if remaining < 0.5:
                move_speed = max(0.1, speed * (remaining / 0.5))
            
            self.robot.drive(move_speed, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
    
    def stop(self):
        self.is_navigating = False
        self.robot.stop()