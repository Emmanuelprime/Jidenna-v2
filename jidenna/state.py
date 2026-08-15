from dataclasses import dataclass
from typing import Optional
import math

@dataclass
class RobotState:
    """
    Represents the current state of the robot
    All angles are in RADIANS unless explicitly stated
    All velocities are in m/s unless explicitly stated
    """
    x: float = 0.0                    # meters
    y: float = 0.0                    # meters
    heading: float = 0.0              # radians (from wheel odometry)
    left_velocity: float = 0.0        # m/s
    right_velocity: float = 0.0       # m/s
    imu_angle_z: float = 0.0          # DEGREES (from MPU6050)
    imu_gyro_z: float = 0.0           # DEGREES/SECOND (from MPU6050)
    timestamp: int = 0                # milliseconds
    
    @property
    def v(self) -> float:
        """Linear velocity in m/s"""
        return (self.left_velocity + self.right_velocity) / 2.0
    
    @property
    def w(self) -> float:
        """Angular velocity in rad/s"""
        WHEEL_SEPARATION = 0.521  # from ESP32 code
        if abs(WHEEL_SEPARATION) < 1e-6:
            return 0.0
        return (self.right_velocity - self.left_velocity) / WHEEL_SEPARATION
    
    @property
    def imu_heading(self) -> float:
        """
        Heading from IMU in RADIANS
        Converts from degrees (ESP32) to radians
        """
        return math.radians(self.imu_angle_z)
    
    @property
    def imu_angular_velocity(self) -> float:
        """
        Angular velocity from IMU in RAD/S
        Converts from deg/s (ESP32) to rad/s
        """
        return math.radians(self.imu_gyro_z)
    
    @property
    def fused_heading(self) -> float:
        """
        Fused heading using simple weighted average
        Weight: 70% IMU, 30% odometry
        Returns heading in RADIANS
        """
        odom_heading = self.heading  # Already in radians
        imu_heading = self.imu_heading  # Converted to radians
        
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
        """
        Difference between odometry and IMU heading
        Returns angle in RADIANS
        """
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
        """Normalize angle to [-pi, pi] (input/output in radians)"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle