import math

class HeadingController:
    def __init__(self, kp=3.5, ki=0.6, kd=0.15, max_correction=0.8):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_correction = max_correction
        self.min_correction = 0.005
        
        self.target_heading = None
        self.integral_error = 0.0
        self.last_error = 0.0
        
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        
    def set_target(self, heading):
        self.target_heading = heading
        self.integral_error = 0.0
        self.last_error = 0.0
        
    def compute(self, current_heading, gyro_rate_rad, dt):
        if self.target_heading is None:
            self.target_heading = current_heading
            return 0.0
        
        # Calculate heading error
        heading_error = self.target_heading - current_heading
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Deadband
        if abs(heading_error) < 0.01:
            heading_error = 0.0
            self.integral_error *= 0.95
        
        # Update integral
        self.integral_error += heading_error * dt
        self.integral_error = max(-2.0, min(2.0, self.integral_error))
        
        # PID terms
        self.p_term = self.kp * heading_error
        self.i_term = self.ki * self.integral_error
        self.d_term = -self.kd * gyro_rate_rad
        
        # Total correction
        correction = self.p_term + self.i_term + self.d_term
        
        # Apply limits
        if abs(correction) < self.min_correction:
            correction = 0.0
        if abs(correction) > self.max_correction:
            correction = max(-self.max_correction, min(self.max_correction, correction))
        
        self.last_error = heading_error
        
        return correction
    
    def get_components(self):
        return self.p_term, self.i_term, self.d_term
    
    def reset(self):
        self.target_heading = None
        self.integral_error = 0.0
        self.last_error = 0.0