import math

class HeadingHoldController:
    def __init__(self, kp=2.5, ki=0.0, kd=0.15, 
                 max_correction=0.5,
                 min_drive_speed=0.05,
                 max_integral=1.0):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_correction = max_correction
        self.min_drive_speed = min_drive_speed
        self.max_integral = max_integral
        
        self.target_heading = None
        self.integral_error = 0.0
        
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.last_error = 0.0
        
    def set_target(self, heading):
        self.target_heading = self._wrap_angle(heading)
        self.integral_error = 0.0
        self.last_error = 0.0
        
    def compute(self, current_heading, gyro_rate_rad, dt, linear_velocity=0.0):
        dt = self._bound_dt(dt)
        
        if self.target_heading is None:
            self.target_heading = self._wrap_angle(current_heading)
            return 0.0
        
        heading_error = self._wrap_angle(self.target_heading - current_heading)
        self.last_error = heading_error
        
        if abs(linear_velocity) > self.min_drive_speed:
            self.integral_error += heading_error * dt
            self.integral_error = max(-self.max_integral, 
                                     min(self.max_integral, self.integral_error))
        
        self.p_term = self.kp * heading_error
        self.i_term = self.ki * self.integral_error
        self.d_term = -self.kd * gyro_rate_rad
        
        correction = self.p_term + self.i_term + self.d_term
        correction = max(-self.max_correction, 
                        min(self.max_correction, correction))
        
        return correction
    
    def _wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def _bound_dt(self, dt):
        if dt <= 0 or dt > 0.1:
            return 0.02
        return dt
    
    def get_components(self):
        return self.p_term, self.i_term, self.d_term
    
    def reset(self):
        self.target_heading = None
        self.integral_error = 0.0
        self.last_error = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0