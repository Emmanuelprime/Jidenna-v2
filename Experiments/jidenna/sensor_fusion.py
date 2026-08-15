import math
from collections import deque

class SensorFusion:
    def __init__(self, filter_time_constant=0.5):
        self.fused_heading = None
        self.last_robot_timestamp = None
        self.gyro_bias = 0.0
        self.gyro_bias_samples = []
        self.gyro_bias_calibration_count = 50
        self.is_calibrating = True
        self.filter_time_constant = filter_time_constant
        self.gyro_filter_buffer = deque(maxlen=5)
        
    def calibrate_gyro(self, gyro_rate_rad):
        if self.is_calibrating:
            self.gyro_bias_samples.append(gyro_rate_rad)
            if len(self.gyro_bias_samples) >= self.gyro_bias_calibration_count:
                self.gyro_bias = sum(self.gyro_bias_samples) / len(self.gyro_bias_samples)
                self.is_calibrating = False
                print(f"Gyro bias calibrated: {self.gyro_bias:.6f} rad/s")
            return True
        return False
    
    def filter_gyro(self, gyro_rate):
        self.gyro_filter_buffer.append(gyro_rate)
        return sum(self.gyro_filter_buffer) / len(self.gyro_filter_buffer)
    
    def update(self, odometry_theta, gyro_rate_rad, timestamp_ms):
        # Handle gyro calibration
        if self.calibrate_gyro(gyro_rate_rad):
            self.last_robot_timestamp = timestamp_ms
            return None
        
        # Correct gyro bias
        gyro_rate_corrected = gyro_rate_rad - self.gyro_bias
        gyro_rate_filtered = self.filter_gyro(gyro_rate_corrected)
        
        # Calculate dt from robot timestamp
        if self.last_robot_timestamp is not None:
            dt = (timestamp_ms - self.last_robot_timestamp) / 1000.0
            if dt <= 0 or dt > 1.0:
                dt = 0.01
        else:
            dt = 0.01
        self.last_robot_timestamp = timestamp_ms
        
        # Initialize heading
        if self.fused_heading is None:
            self.fused_heading = odometry_theta
            return self.fused_heading
        
        # Complementary filter
        gyro_heading = self.fused_heading + gyro_rate_filtered * dt
        alpha = self.filter_time_constant / (self.filter_time_constant + dt)
        self.fused_heading = alpha * gyro_heading + (1 - alpha) * odometry_theta
        
        # Normalize
        while self.fused_heading > math.pi:
            self.fused_heading -= 2 * math.pi
        while self.fused_heading < -math.pi:
            self.fused_heading += 2 * math.pi
        
        return self.fused_heading
    
    def reset(self):
        self.fused_heading = None
        self.last_robot_timestamp = None
        self.gyro_bias = 0.0
        self.gyro_bias_samples = []
        self.is_calibrating = True
        self.gyro_filter_buffer.clear()