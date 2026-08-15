import math
import time

class G2GController:
    def __init__(self, robot, heading_controller, turning_controller=None):
        self.robot = robot
        self.heading_controller = heading_controller
        self.turning_controller = turning_controller
        
        self.linear_speed = 0.2
        self.distance_tolerance = 0.15
        self.heading_tolerance = 0.1
        self.timeout = 25
        
        self.turn_kp = 1.5
        self.turn_ki = 0.0
        self.turn_kd = 0.3
        self.max_turn_speed = 0.6
        self.min_turn_speed = 0.08
        self.turn_integral = 0.0
        self.last_turn_error = 0.0
        self.last_turn_time = None
        
        self.is_navigating = False
        self.goal_reached = False
        self.turn_direction_locked = False
        self.turn_direction = 1  # 1 = left, -1 = right
        
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
    
    def _turn_to_heading(self, target_heading, timeout=6):
        """Turn in place to target heading with direction lock"""
        self.robot.stop()
        time.sleep(0.3)
        
        print(f"  Turning to {target_heading:.2f} rad ({target_heading*180/math.pi:.0f} deg)...")
        
        # Lock turn direction at the start
        current_heading = self.get_heading(timeout=2)
        if current_heading is None:
            return
        
        heading_error = self.shortest_angle_error(target_heading, current_heading)
        
        # Choose turn direction (shortest path)
        if heading_error > 0:
            self.turn_direction = 1  # Turn left
        else:
            self.turn_direction = -1  # Turn right
        
        print(f"  Turn direction: {'LEFT' if self.turn_direction > 0 else 'RIGHT'}")
        
        start_time = time.time()
        last_time = time.time()
        self.turn_integral = 0.0
        
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
            
            heading_error = self.shortest_angle_error(target_heading, current_heading)
            
            # Check if turn complete
            if abs(heading_error) < self.heading_tolerance:
                print(f"  Turn complete! Error: {heading_error:.3f} rad ({heading_error*180/math.pi:.1f} deg)")
                break
            
            # Use fixed turn direction with speed based on error
            abs_error = abs(heading_error)
            
            if abs_error > 0.5:  # More than 28 degrees
                # Turn fast
                w = self.turn_direction * self.max_turn_speed
            elif abs_error > self.heading_tolerance:
                # Slow down proportionally
                speed_factor = abs_error / 0.5
                w = self.turn_direction * self.max_turn_speed * speed_factor
                # Maintain minimum speed
                if abs(w) < self.min_turn_speed:
                    w = self.turn_direction * self.min_turn_speed
            else:
                # Close enough
                w = 0.0
            
            self.robot.drive(0, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
    
    def _move_straight(self, distance, heading, speed=0.2, timeout=15):
        """Move straight for a specific distance"""
        self.robot.stop()
        time.sleep(0.2)
        
        self.heading_controller.set_target(heading)
        start_time = time.time()
        start_pose = self.get_pose(timeout=2)
        
        if start_pose is None:
            return False
        
        start_x, start_y, _ = start_pose
        moved_distance = 0
        last_print_time = 0
        
        print(f"  Moving {distance:.2f}m...")
        
        while moved_distance < distance - self.distance_tolerance:
            if time.time() - start_time > timeout:
                print("  Move timeout!")
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
            if remaining < 0.3:
                move_speed = max(0.08, speed * (remaining / 0.3))
            
            if time.time() - last_print_time > 1:
                print(f"    Dist: {moved_distance:.2f}m / {distance:.2f}m, Remaining: {remaining:.2f}m")
                last_print_time = time.time()
            
            self.robot.drive(move_speed, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
        print(f"  Move complete! Final distance: {moved_distance:.2f}m")
        return True
    
    def go_to_goal(self, goal_x, goal_y, speed=None):
        """Navigate to goal using unified controller"""
        if speed:
            self.linear_speed = speed
        
        self.is_navigating = True
        self.goal_reached = False
        
        print(f"\nGoing to goal: ({goal_x:.2f}, {goal_y:.2f})")
        
        target_heading = math.atan2(goal_y, goal_x)
        distance = math.sqrt(goal_x**2 + goal_y**2)
        
        # Phase 1: Turn to face goal
        current_heading = self.get_heading(timeout=5)
        if current_heading is not None:
            heading_error = self.shortest_angle_error(target_heading, current_heading)
            if abs(heading_error) > self.heading_tolerance:
                self._turn_to_heading(target_heading)
        
        # Phase 2: Move straight
        success = self._move_straight(distance, target_heading, self.linear_speed)
        
        self.is_navigating = False
        self.goal_reached = success
        
        return success
    
    def go_to_goal_with_phases(self, goal_x, goal_y, speed=None):
        """Alias for go_to_goal - same unified controller"""
        return self.go_to_goal(goal_x, goal_y, speed)
    
    def stop(self):
        self.is_navigating = False
        self.robot.stop()