import math

class TurningController:
    def __init__(self, kp=2.5, ki=0.05, kd=0.4, max_angular_velocity=0.8):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_angular_velocity = max_angular_velocity
        
        self.target_heading = None
        self.integral_error = 0.0
        self.last_error = 0.0
        
        self.slowdown_threshold = 0.3
        self.stop_threshold = 0.05
        
    def set_target(self, heading):
        self.target_heading = heading
        self.integral_error = 0.0
        self.last_error = 0.0
        
    def shortest_angle_error(self, target, current):
        """Calculate shortest angle error, always returns value between -pi and pi"""
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
        
        # Calculate shortest angle error
        heading_error = self.shortest_angle_error(self.target_heading, current_heading)
        
        # Update integral (only accumulate when error is small to prevent windup)
        if abs(heading_error) < 0.5:
            self.integral_error += heading_error * dt
            self.integral_error = max(-0.5, min(0.5, self.integral_error))
        else:
            self.integral_error = 0.0
        
        # P term
        p_term = self.kp * heading_error
        
        # I term
        i_term = self.ki * self.integral_error
        
        # D term - use gyro rate for damping
        d_term = -self.kd * gyro_rate_rad
        
        # Total angular velocity
        angular_velocity = p_term + i_term + d_term
        
        # Apply slowdown near target
        abs_error = abs(heading_error)
        if abs_error < self.slowdown_threshold:
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