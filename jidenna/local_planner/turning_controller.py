import math
import time

class TurningController:
    def __init__(self, kp=2.0, ki=0.1, kd=0.3, max_angular_velocity=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_angular_velocity = max_angular_velocity
        
        self.target_heading = None
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = None
        
        self.slowdown_threshold = 0.3  # Start slowing down at 17 degrees from target
        self.stop_threshold = 0.05     # Stop at 3 degrees from target
        
    def set_target(self, heading):
        self.target_heading = heading
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = None
        
    def shortest_angle_error(self, target, current):
        error = target - current
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        return error
    
    def compute(self, current_heading, gyro_rate_rad, dt):
        if self.target_heading is None:
            self.target_heading = current_heading
            return 0.0
        
        # Calculate heading error (shortest path)
        heading_error = self.shortest_angle_error(self.target_heading, current_heading)
        
        # Update integral (only when not saturated)
        self.integral_error += heading_error * dt
        self.integral_error = max(-1.0, min(1.0, self.integral_error))
        
        # P term
        p_term = self.kp * heading_error
        
        # I term
        i_term = self.ki * self.integral_error
        
        # D term - use gyro rate directly
        d_term = -self.kd * gyro_rate_rad
        
        # Total angular velocity
        angular_velocity = p_term + i_term + d_term
        
        # Apply slowdown near target
        abs_error = abs(heading_error)
        if abs_error < self.slowdown_threshold:
            # Scale down angular velocity as we approach target
            slowdown_factor = abs_error / self.slowdown_threshold
            angular_velocity *= slowdown_factor
        
        # Limit angular velocity
        if abs(angular_velocity) > self.max_angular_velocity:
            angular_velocity = self.max_angular_velocity * (1 if angular_velocity > 0 else -1)
        
        # Deadband
        if abs(angular_velocity) < 0.01:
            angular_velocity = 0.0
        
        self.last_error = heading_error
        
        return angular_velocity
    
    def is_turn_complete(self, current_heading):
        if self.target_heading is None:
            return True
        
        error = self.shortest_angle_error(self.target_heading, current_heading)
        return abs(error) < self.stop_threshold
    
    def reset(self):
        self.target_heading = None
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = None