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
        return math.atan2(math.sin(target - current), math.cos(target - current))
    
    def _turn_to_heading(self, target_heading, timeout=6):
        self.robot.stop()
        time.sleep(0.3)
        
        print(f"  Turning to {math.degrees(target_heading):.0f}°...")
        
        start_time = time.time()
        last_time = time.time()
        last_error = None
        min_error = float('inf')
        
        while time.time() - start_time < timeout:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            current_heading = self.get_heading(timeout=1)
            if current_heading is None:
                continue
            
            data = self.robot.serial.get_latest_data()
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
            
            heading_error = self.shortest_angle_error(target_heading, current_heading)
            abs_error = abs(heading_error)
            
            # Track minimum error to detect overshoot
            if abs_error < min_error:
                min_error = abs_error
            
            # Check if turn complete or overshooting
            if abs_error < self.heading_tolerance:
                print(f"  Turn complete! Error: {math.degrees(heading_error):.1f}°")
                break
            
            # Detect overshoot (error started increasing after getting close)
            if min_error < 0.3 and abs_error > min_error * 1.5:
                print(f"  Overshoot detected, stopping turn")
                break
            
            # Simple proportional control with minimum speed
            if abs_error > 0.5:
                w = self.max_turn_speed * (1 if heading_error > 0 else -1)
            else:
                # Proportional slowdown
                w = self.turn_kp * heading_error
                # Enforce minimum speed if still turning
                if abs(w) < self.min_turn_speed and abs_error > self.heading_tolerance:
                    w = self.min_turn_speed * (1 if heading_error > 0 else -1)
                # Clamp to max
                w = max(-self.max_turn_speed, min(self.max_turn_speed, w))
            
            self.robot.drive(0, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
    
    def _move_straight(self, distance, heading, speed=0.2, timeout=15):
        self.robot.stop()
        time.sleep(0.2)
        
        self.heading_controller.set_target(heading)
        start_time = time.time()
        last_time = time.time()
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
            
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            pose = self.get_pose(timeout=1)
            if pose is None:
                continue
            
            current_x, current_y, _ = pose
            moved_distance = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            
            current_heading = self.get_heading(timeout=1)
            if current_heading is None:
                continue
            
            data = self.robot.serial.get_latest_data()
            gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0 if data else 0.0
            
            remaining = distance - moved_distance
            move_speed = speed
            if remaining < 0.3:
                move_speed = max(0.08, speed * (remaining / 0.3))
            
            # Fixed: pass actual dt and linear_velocity
            w = self.heading_controller.compute(
                current_heading, 
                gyro_rate_rad, 
                dt,
                linear_velocity=move_speed
            )
            
            if current_time - last_print_time > 1:
                print(f"    Dist: {moved_distance:.2f}m / {distance:.2f}m")
                last_print_time = current_time
            
            self.robot.drive(move_speed, w)
            time.sleep(0.01)
        
        self.robot.stop()
        time.sleep(0.3)
        print(f"  Move complete! Final distance: {moved_distance:.2f}m")
        return True
    
    def go_to_goal(self, goal_x, goal_y, speed=None):
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
    
    def stop(self):
        self.is_navigating = False
        self.robot.stop()