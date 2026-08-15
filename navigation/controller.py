import math
from typing import Tuple

class SpeedController:
    """Controls robot speed based on path curvature and distance to goal"""
    
    def __init__(self, max_linear_velocity: float = 1.0,
                 max_angular_velocity: float = 2.0,
                 min_linear_velocity: float = 0.0,
                 curvature_speed_factor: float = 0.5):
        """
        Initialize speed controller
        
        Args:
            max_linear_velocity: Maximum linear velocity (m/s)
            max_angular_velocity: Maximum angular velocity (rad/s)
            min_linear_velocity: Minimum linear velocity (m/s)
            curvature_speed_factor: How much to reduce speed based on curvature
        """
        self.max_v = max_linear_velocity
        self.max_w = max_angular_velocity
        self.min_v = min_linear_velocity
        self.curvature_factor = curvature_speed_factor
    
    def compute_velocity(self, target_v: float, curvature: float, 
                        distance_to_goal: float = float('inf'),
                        goal_slowdown_distance: float = 1.0) -> Tuple[float, float]:
        """
        Compute velocity commands with speed limiting
        
        Args:
            target_v: Desired linear velocity (m/s)
            curvature: Path curvature (1/m)
            distance_to_goal: Distance to goal (m)
            goal_slowdown_distance: Start slowing down within this distance to goal
        
        Returns:
            (v, w) - limited linear and angular velocity
        """
        # Start with target velocity
        v = target_v
        
        # Limit based on curvature (reduce speed for sharp turns)
        if abs(curvature) > 0.01:
            # Maximum safe velocity for given curvature
            # Based on maximum lateral acceleration of 0.5 m/s²
            max_lateral_accel = 0.5
            v_curvature = math.sqrt(max_lateral_accel / abs(curvature))
            v = min(v, v_curvature)
        
        # Slow down when approaching goal
        if distance_to_goal < goal_slowdown_distance:
            slowdown_factor = max(0.1, distance_to_goal / goal_slowdown_distance)
            v *= slowdown_factor
        
        # Apply limits
        v = max(self.min_v, min(v, self.max_v))
        
        # Calculate angular velocity
        w = v * curvature
        
        # Limit angular velocity
        w = max(-self.max_w, min(w, self.max_w))
        
        return v, w