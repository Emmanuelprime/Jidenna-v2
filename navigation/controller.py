import math
from typing import Tuple

class SpeedController:
    """Controls robot speed based on path curvature and distance to goal"""
    
    def __init__(self, max_linear_velocity: float = 1.0,
                 max_angular_velocity: float = 2.0,
                 min_linear_velocity: float = 0.0,
                 curvature_speed_factor: float = 0.5,
                 max_lateral_acceleration: float = 0.5,
                 max_linear_acceleration: float = 0.5,
                 max_angular_acceleration: float = 1.0):
        """
        Initialize speed controller
        
        Args:
            max_linear_velocity: Maximum linear velocity (m/s)
            max_angular_velocity: Maximum angular velocity (rad/s)
            min_linear_velocity: Minimum linear velocity (m/s)
            curvature_speed_factor: How much to reduce speed based on curvature
            max_lateral_acceleration: Maximum lateral acceleration (m/s²)
            max_linear_acceleration: Maximum linear acceleration (m/s²)
            max_angular_acceleration: Maximum angular acceleration (rad/s²)
        """
        self.max_v = max_linear_velocity
        self.max_w = max_angular_velocity
        self.min_v = min_linear_velocity
        self.curvature_factor = curvature_speed_factor
        self.max_lateral_accel = max_lateral_acceleration
        self.max_linear_accel = max_linear_acceleration
        self.max_angular_accel = max_angular_acceleration
        
        # For acceleration limiting
        self.last_v = 0.0
        self.last_w = 0.0
        self.last_time = None
    
    def compute_velocity(self, target_v: float, curvature: float, 
                        distance_to_goal: float = float('inf'),
                        goal_slowdown_distance: float = 1.0,
                        dt: float = 0.05) -> Tuple[float, float]:
        """
        Compute velocity commands with speed limiting
        
        Args:
            target_v: Desired linear velocity (m/s)
            curvature: Path curvature (1/m)
            distance_to_goal: Distance to goal (m)
            goal_slowdown_distance: Start slowing down within this distance to goal
            dt: Time step for acceleration limiting (seconds)
        
        Returns:
            (v, w) - limited linear and angular velocity
        """
        # Start with target velocity
        v = target_v
        
        # Limit based on curvature (reduce speed for sharp turns)
        if abs(curvature) > 0.01:
            # Maximum safe velocity for given curvature
            # Based on maximum lateral acceleration
            v_curvature = math.sqrt(self.max_lateral_accel / abs(curvature))
            v = min(v, v_curvature)
            
            # Additional reduction for very sharp turns
            if abs(curvature) > 1.0:
                v *= 0.7  # Extra 30% reduction for sharp turns
            elif abs(curvature) > 0.5:
                v *= 0.85  # Extra 15% reduction for moderate turns
        
        # Slow down when approaching goal
        if distance_to_goal < goal_slowdown_distance:
            # Smooth deceleration using quadratic profile
            slowdown_factor = (distance_to_goal / goal_slowdown_distance) ** 1.5
            slowdown_factor = max(0.05, min(1.0, slowdown_factor))
            v *= slowdown_factor
        
        # Apply hard limits
        v = max(self.min_v, min(v, self.max_v))
        
        # Apply acceleration limiting
        v = self._limit_acceleration(v, self.max_linear_accel, dt)
        
        # Calculate angular velocity
        w = v * curvature
        
        # Limit angular velocity
        w = max(-self.max_w, min(w, self.max_w))
        
        # Apply angular acceleration limiting
        w = self._limit_angular_acceleration(w, self.max_angular_accel, dt)
        
        # Store for next iteration
        self.last_v = v
        self.last_w = w
        
        return v, w
    
    def _limit_acceleration(self, target_v: float, max_accel: float, dt: float) -> float:
        """
        Limit linear acceleration to prevent jerky motion
        
        Args:
            target_v: Target velocity
            max_accel: Maximum acceleration (m/s²)
            dt: Time step (seconds)
        
        Returns:
            Velocity with acceleration limiting
        """
        if dt <= 0:
            return target_v
        
        # Calculate maximum change in velocity
        max_delta = max_accel * dt
        
        # Limit change from last velocity
        delta = target_v - self.last_v
        
        if abs(delta) > max_delta:
            # Limit the change
            limited_delta = math.copysign(max_delta, delta)
            return self.last_v + limited_delta
        
        return target_v
    
    def _limit_angular_acceleration(self, target_w: float, max_accel: float, dt: float) -> float:
        """
        Limit angular acceleration to prevent jerky rotation
        
        Args:
            target_w: Target angular velocity
            max_accel: Maximum angular acceleration (rad/s²)
            dt: Time step (seconds)
        
        Returns:
            Angular velocity with acceleration limiting
        """
        if dt <= 0:
            return target_w
        
        # Calculate maximum change in angular velocity
        max_delta = max_accel * dt
        
        # Limit change from last angular velocity
        delta = target_w - self.last_w
        
        if abs(delta) > max_delta:
            # Limit the change
            limited_delta = math.copysign(max_delta, delta)
            return self.last_w + limited_delta
        
        return target_w
    
    def reset(self):
        """Reset acceleration limiting state"""
        self.last_v = 0.0
        self.last_w = 0.0
        self.last_time = None
    
    def get_debug_info(self) -> dict:
        """Get debug information about speed controller"""
        return {
            'max_v': self.max_v,
            'max_w': self.max_w,
            'current_v': self.last_v,
            'current_w': self.last_w,
            'max_lateral_accel': self.max_lateral_accel,
            'max_linear_accel': self.max_linear_accel,
            'max_angular_accel': self.max_angular_accel
        }