from dataclasses import dataclass
from typing import Optional
import math

@dataclass
class RobotState:
    """Represents the current state of the robot"""
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians (from odometry)
    left_velocity: float = 0.0  # m/s
    right_velocity: float = 0.0  # m/s
    imu_angle_z: float = 0.0  # degrees (from MPU6050)
    imu_gyro_z: float = 0.0  # deg/s (from MPU6050)
    timestamp: int = 0  # milliseconds
    
    @property
    def v(self) -> float:
        """Linear velocity in m/s"""
        return (self.left_velocity + self.right_velocity) / 2.0
    
    @property
    def w(self) -> float:
        """Angular velocity in rad/s"""
        # Differential drive: w = (vr - vl) / wheel_separation
        WHEEL_SEPARATION = 0.521  # from ESP32 code
        if abs(WHEEL_SEPARATION) < 1e-6:
            return 0.0
        return (self.right_velocity - self.left_velocity) / WHEEL_SEPARATION
    
    @property
    def imu_heading(self) -> float:
        """Heading from IMU in radians"""
        return math.radians(self.imu_angle_z)
    
    @property
    def imu_angular_velocity(self) -> float:
        """Angular velocity from IMU in rad/s"""
        return math.radians(self.imu_gyro_z)
    
    @property
    def fused_heading(self) -> float:
        """
        Fused heading using simple weighted average
        Weight: 70% IMU, 30% odometry (IMU is more reliable for short-term heading)
        """
        # Handle angle wrapping
        odom_heading = self.heading
        imu_heading = self.imu_heading
        
        # Calculate shortest angular difference
        diff = self._normalize_angle(odom_heading - imu_heading)
        
        # If discrepancy is too large (> 30 degrees), trust odometry more
        if abs(diff) > math.radians(30):
            return odom_heading
        
        # Weighted average (70% IMU, 30% odometry)
        fused = imu_heading + 0.3 * diff
        
        return self._normalize_angle(fused)
    
    @property
    def heading_discrepancy(self) -> float:
        """Difference between odometry and IMU heading (radians)"""
        return self._normalize_angle(self.heading - self.imu_heading)
    
    def is_valid(self) -> bool:
        """Check if state contains valid data"""
        return (math.isfinite(self.x) and 
                math.isfinite(self.y) and 
                math.isfinite(self.heading) and
                math.isfinite(self.left_velocity) and
                math.isfinite(self.right_velocity))
    
    def is_imu_valid(self) -> bool:
        """Check if IMU data is valid"""
        return (math.isfinite(self.imu_angle_z) and 
                math.isfinite(self.imu_gyro_z))
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle