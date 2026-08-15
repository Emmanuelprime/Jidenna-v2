import math
import numpy as np
from typing import Tuple, Optional
import time
import sys
import os

# Add parent directory to path if needed
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from jidenna.state import RobotState

class DifferentialDriveSimulator:
    """Simple differential drive robot simulator"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0,
                 wheel_separation: float = 0.521, wheel_diameter: float = 0.165,
                 max_linear_velocity: float = 1.0, max_angular_velocity: float = 2.0,
                 noise_std: float = 0.01):
        """
        Initialize simulator
        
        Args:
            x, y, theta: Initial pose
            wheel_separation: Distance between wheels (m)
            wheel_diameter: Wheel diameter (m)
            max_linear_velocity: Maximum linear velocity (m/s)
            max_angular_velocity: Maximum angular velocity (rad/s)
            noise_std: Standard deviation of Gaussian noise for odometry
        """
        self.x = x
        self.y = y
        self.theta = theta
        
        self.wheel_separation = wheel_separation
        self.wheel_diameter = wheel_diameter
        self.max_v = max_linear_velocity
        self.max_w = max_angular_velocity
        self.noise_std = noise_std
        
        self.v = 0.0
        self.w = 0.0
        
        self.trajectory = [(x, y, theta)]
        self.time = 0.0
    
    def set_velocity(self, v: float, w: float):
        """Set velocity command"""
        # Apply limits
        self.v = max(-self.max_v, min(v, self.max_v))
        self.w = max(-self.max_w, min(w, self.max_w))
    
    def update(self, dt: float):
        """Update robot pose based on current velocities"""
        if dt <= 0:
            return
        
        # Add noise to velocities (simulating real-world uncertainty)
        v_noisy = self.v + np.random.normal(0, self.noise_std)
        w_noisy = self.w + np.random.normal(0, self.noise_std * 0.5)
        
        # Update pose using differential drive kinematics
        if abs(w_noisy) < 1e-6:
            # Straight line motion
            self.x += v_noisy * math.cos(self.theta) * dt
            self.y += v_noisy * math.sin(self.theta) * dt
        else:
            # Arc motion
            self.x += (v_noisy / w_noisy) * (math.sin(self.theta + w_noisy * dt) - math.sin(self.theta))
            self.y -= (v_noisy / w_noisy) * (math.cos(self.theta + w_noisy * dt) - math.cos(self.theta))
            self.theta += w_noisy * dt
        
        # Normalize theta
        self.theta = self.normalize_angle(self.theta)
        
        # Add small random drift
        self.x += np.random.normal(0, self.noise_std * 0.1)
        self.y += np.random.normal(0, self.noise_std * 0.1)
        
        self.time += dt
        self.trajectory.append((self.x, self.y, self.theta))
    
    def get_pose(self) -> Tuple[float, float, float]:
        """Get current robot pose"""
        return (self.x, self.y, self.theta)
    
    def get_odometry(self) -> Tuple[float, float, float, float, float]:
        """Get odometry data with simulated noise
        
        Returns:
            (x, y, theta, left_vel, right_vel)
        """
        # Calculate wheel velocities from robot velocity
        v_l = self.v - (self.w * self.wheel_separation) / 2.0
        v_r = self.v + (self.w * self.wheel_separation) / 2.0
        
        # Add noise
        v_l += np.random.normal(0, self.noise_std)
        v_r += np.random.normal(0, self.noise_std)
        
        return (self.x, self.y, self.theta, v_l, v_r)
    
    def get_trajectory(self):
        """Get trajectory history"""
        return self.trajectory
    
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset simulator"""
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.w = 0.0
        self.time = 0.0
        self.trajectory = [(x, y, theta)]
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class SimulatedRobotAPI:
    """Simulated robot API that mimics the real RobotAPI interface"""
    
    def __init__(self, simulator: DifferentialDriveSimulator = None):
        """Initialize simulated robot API"""
        self.simulator = simulator if simulator else DifferentialDriveSimulator()
        self.connected = False
        self.update_rate = 0.05  # 20 Hz (matching ESP32)
    
    def connect(self) -> bool:
        """Simulate connection"""
        self.connected = True
        return True
    
    def disconnect(self):
        """Simulate disconnection"""
        self.stop()
        self.connected = False
    
    def is_connected(self) -> bool:
        """Check if connected"""
        return self.connected
    
    def set_velocity(self, v: float, w: float) -> bool:
        """Set velocity command"""
        if not self.connected:
            return False
        
        self.simulator.set_velocity(v, w)
        # Simulate immediate execution
        self.simulator.update(self.update_rate)
        return True
    
    def stop(self) -> bool:
        """Stop robot"""
        return self.set_velocity(0.0, 0.0)
    
    def get_state(self):
        """Get simulated robot state"""
        x, y, theta, vl, vr = self.simulator.get_odometry()
        
        state = RobotState(
            x=x,
            y=y,
            heading=theta,
            left_velocity=vl,
            right_velocity=vr,
            imu_angle_z=math.degrees(theta),  # Simulate IMU
            imu_gyro_z=0.0,
            timestamp=int(time.time() * 1000)
        )
        return state
    
    def update(self):
        """Update simulator"""
        self.simulator.update(self.update_rate)