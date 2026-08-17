from dataclasses import dataclass, field
from typing import Optional, List, Dict
import math
import time

@dataclass
class RobotState:
    """
    Represents the current state of the robot
    All angles are in RADIANS unless explicitly stated
    All velocities are in m/s unless explicitly stated
    All distances are in METERS unless explicitly stated
    """
    # Odometry
    x: float = 0.0                    # meters
    y: float = 0.0                    # meters
    heading: float = 0.0              # radians (from wheel odometry)
    left_velocity: float = 0.0        # m/s
    right_velocity: float = 0.0       # m/s
    
    # IMU
    imu_angle_z: float = 0.0          # DEGREES (from MPU6050)
    imu_gyro_z: float = 0.0           # DEGREES/SECOND (from MPU6050)
    
    # Ultrasonic sensors (distances in METERS)
    ultrasonic_left: float = -1.0     # meters (-1 = no reading)
    ultrasonic_center: float = -1.0   # meters
    ultrasonic_right: float = -1.0    # meters
    
    # Timestamps
    timestamp: int = 0                # milliseconds (from motor ESP32)
    ultrasonic_timestamp: int = 0     # milliseconds (from ultrasonic ESP32)
    
    # Internal tracking
    _last_ultrasonic_update: float = field(default=0.0, repr=False)  # System time of last update
    _ultrasonic_timeout: float = field(default=2.0, repr=False)      # Timeout in seconds
    
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
        """Heading from IMU in RADIANS"""
        return math.radians(self.imu_angle_z)
    
    @property
    def imu_angular_velocity(self) -> float:
        """Angular velocity from IMU in RAD/S"""
        return math.radians(self.imu_gyro_z)
    
    @property
    def fused_heading(self) -> float:
        """
        Fused heading using simple weighted average
        Weight: 70% IMU, 30% odometry
        Returns heading in RADIANS
        """
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
        """Difference between odometry and IMU heading (RADIANS)"""
        return self._normalize_angle(self.heading - self.imu_heading)
    
    @property
    def has_ultrasonic_data(self) -> bool:
        """Check if ultrasonic data is available"""
        return self.ultrasonic_timestamp > 0
    
    @property
    def ultrasonic_data_fresh(self) -> bool:
        """Check if ultrasonic data is fresh (not timed out)"""
        if not self.has_ultrasonic_data:
            return False
        return (time.time() - self._last_ultrasonic_update) < self._ultrasonic_timeout
    
    @property
    def min_ultrasonic_distance(self) -> float:
        """Minimum ultrasonic distance (meters)"""
        valid = self.get_valid_ultrasonic_distances()
        return min(valid) if valid else -1.0
    
    @property
    def max_ultrasonic_distance(self) -> float:
        """Maximum ultrasonic distance (meters)"""
        valid = self.get_valid_ultrasonic_distances()
        return max(valid) if valid else -1.0
    
    @property
    def ultrasonic_readings(self) -> Dict[str, float]:
        """Get all ultrasonic readings as dictionary (meters)"""
        return {
            'left': self.ultrasonic_left,
            'center': self.ultrasonic_center,
            'right': self.ultrasonic_right
        }
    
    def update_ultrasonic(self, left_m: float, center_m: float, right_m: float, 
                         timestamp: int = 0):
        """
        Update ultrasonic sensor data
        
        Args:
            left_m: Left sensor distance in meters (-1 if invalid)
            center_m: Center sensor distance in meters (-1 if invalid)
            right_m: Right sensor distance in meters (-1 if invalid)
            timestamp: ESP32 timestamp in milliseconds
        """
        # Validate inputs
        self.ultrasonic_left = self._validate_distance(left_m)
        self.ultrasonic_center = self._validate_distance(center_m)
        self.ultrasonic_right = self._validate_distance(right_m)
        
        if timestamp > 0:
            self.ultrasonic_timestamp = timestamp
        
        # Update last update time
        self._last_ultrasonic_update = time.time()
    
    def update_from_ultrasonic_data(self, ultrasonic_data):
        """
        Update from UltrasonicData object
        
        Args:
            ultrasonic_data: UltrasonicData instance
        """
        if ultrasonic_data and ultrasonic_data.is_valid():
            self.update_ultrasonic(
                ultrasonic_data.left,
                ultrasonic_data.center,
                ultrasonic_data.right,
                ultrasonic_data.timestamp
            )
    
    def get_valid_ultrasonic_distances(self) -> List[float]:
        """Get list of valid ultrasonic distances (meters)"""
        return [d for d in [self.ultrasonic_left, 
                           self.ultrasonic_center, 
                           self.ultrasonic_right] if d > 0]
    
    def get_ultrasonic_reading(self, sensor: str) -> float:
        """
        Get reading from specific ultrasonic sensor
        
        Args:
            sensor: 'left', 'center', or 'right'
        
        Returns:
            Distance in meters, or -1 if invalid
        """
        if sensor.lower() == 'left':
            return self.ultrasonic_left
        elif sensor.lower() == 'center':
            return self.ultrasonic_center
        elif sensor.lower() == 'right':
            return self.ultrasonic_right
        else:
            raise ValueError(f"Unknown sensor: {sensor}")
    
    def is_valid(self) -> bool:
        """Check if state contains valid odometry data"""
        return (math.isfinite(self.x) and 
                math.isfinite(self.y) and 
                math.isfinite(self.heading) and
                math.isfinite(self.left_velocity) and
                math.isfinite(self.right_velocity))
    
    def is_imu_valid(self) -> bool:
        """Check if IMU data is valid"""
        return (math.isfinite(self.imu_angle_z) and 
                math.isfinite(self.imu_gyro_z))
    
    def is_ultrasonic_valid(self) -> bool:
        """Check if ultrasonic data is valid and fresh"""
        if not self.has_ultrasonic_data:
            return False
        
        # Check if any sensor has valid reading
        has_valid_reading = any(d > 0 for d in [self.ultrasonic_left, 
                                                self.ultrasonic_center, 
                                                self.ultrasonic_right])
        
        # Check if data is fresh
        is_fresh = self.ultrasonic_data_fresh
        
        return has_valid_reading and is_fresh
    
    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            # Odometry
            'x': self.x,
            'y': self.y,
            'heading': self.heading,
            'left_velocity': self.left_velocity,
            'right_velocity': self.right_velocity,
            'v': self.v,
            'w': self.w,
            # IMU
            'imu_angle_z': self.imu_angle_z,
            'imu_gyro_z': self.imu_gyro_z,
            'imu_heading': self.imu_heading,
            # Ultrasonic
            'ultrasonic_left': self.ultrasonic_left,
            'ultrasonic_center': self.ultrasonic_center,
            'ultrasonic_right': self.ultrasonic_right,
            'min_ultrasonic_distance': self.min_ultrasonic_distance,
            # Timestamps
            'timestamp': self.timestamp,
            'ultrasonic_timestamp': self.ultrasonic_timestamp,
            # Status
            'is_valid': self.is_valid(),
            'is_imu_valid': self.is_imu_valid(),
            'is_ultrasonic_valid': self.is_ultrasonic_valid()
        }
    
    def __str__(self) -> str:
        """String representation"""
        return (f"RobotState(pos=({self.x:.2f}, {self.y:.2f}), "
                f"heading={math.degrees(self.heading):.1f}°, "
                f"v={self.v:.2f}m/s, "
                f"ultra=[L:{self.ultrasonic_left:.2f}m, "
                f"C:{self.ultrasonic_center:.2f}m, "
                f"R:{self.ultrasonic_right:.2f}m])")
    
    def _validate_distance(self, distance: float) -> float:
        """
        Validate distance value
        
        Args:
            distance: Distance in meters
        
        Returns:
            Validated distance, or -1 if invalid
        """
        if distance is None or not math.isfinite(distance):
            return -1.0
        
        # Valid range: 0.02m (2cm) to 5.0m (500cm)
        if distance < 0:
            return -1.0
        elif distance > 5.0:
            return -1.0
        else:
            return distance
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi] (input/output in radians)"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle