from dataclasses import dataclass
from typing import Optional
import math

@dataclass
class RobotState:
    """Represents the current state of the robot"""
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians
    left_velocity: float = 0.0  # m/s
    right_velocity: float = 0.0  # m/s
    imu_angle_z: float = 0.0  # degrees
    imu_gyro_z: float = 0.0  # deg/s
    timestamp: int = 0  # milliseconds
    
    @property
    def v(self) -> float:
        """Linear velocity in m/s"""
        return (self.left_velocity + self.right_velocity) / 2.0
    
    @property
    def w(self) -> float:
        """Angular velocity in rad/s"""
        # Differential drive: w = (vr - vl) / wheel_separation
        # We'll approximate with a standard separation if not known
        WHEEL_SEPARATION = 0.521  # from ESP32 code
        if abs(WHEEL_SEPARATION) < 1e-6:
            return 0.0
        return (self.right_velocity - self.left_velocity) / WHEEL_SEPARATION
    
    def is_valid(self) -> bool:
        """Check if state contains valid data"""
        return (math.isfinite(self.x) and 
                math.isfinite(self.y) and 
                math.isfinite(self.heading) and
                math.isfinite(self.left_velocity) and
                math.isfinite(self.right_velocity))