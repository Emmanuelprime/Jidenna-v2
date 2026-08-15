import math
from typing import Tuple, Optional
import numpy as np
from .path import Path

class PurePursuit:
    """Pure Pursuit path tracking algorithm for differential drive robots"""
    
    def __init__(self, lookahead_distance: float = 0.5, 
                 lookahead_min: float = 0.3,
                 lookahead_max: float = 1.5):
        """
        Initialize Pure Pursuit controller
        
        Args:
            lookahead_distance: Base lookahead distance in meters
            lookahead_min: Minimum lookahead distance
            lookahead_max: Maximum lookahead distance
        """
        self.lookahead_distance = lookahead_distance
        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max
        self.current_lookahead = lookahead_distance
        
    def compute_curvature(self, robot_pose: Tuple[float, float, float], 
                         target_point: Tuple[float, float]) -> float:
        """
        Compute curvature to reach target point
        
        Args:
            robot_pose: (x, y, heading) in world frame
            target_point: (x, y) target in world frame
        
        Returns:
            curvature (1/m)
        """
        x, y, heading = robot_pose
        tx, ty = target_point
        
        # Transform target to robot frame
        dx = tx - x
        dy = ty - y
        
        # Rotate to robot frame
        local_x = dx * math.cos(heading) + dy * math.sin(heading)
        local_y = -dx * math.sin(heading) + dy * math.cos(heading)
        
        # Calculate curvature
        distance_sq = local_x**2 + local_y**2
        if distance_sq < 1e-6:  # Target is at robot position
            return 0.0
        
        curvature = 2.0 * local_y / distance_sq
        
        return curvature
    
    def select_lookahead_point(self, path: Path, robot_pose: Tuple[float, float, float],
                              current_index: int) -> Tuple[Optional[Tuple[float, float]], int]:
        """
        Select lookahead point on path
        
        Args:
            path: Path object
            robot_pose: (x, y, heading)
            current_index: Current nearest point index on path
        
        Returns:
            (lookahead_point, lookahead_index) - point and its index on path
        """
        # Dynamic lookahead based on speed
        lookahead = self.current_lookahead
        
        # Try to find point at lookahead distance
        lookahead_point = path.get_point_at_distance(current_index, lookahead)
        
        if lookahead_point is None:
            # Path too short, use last point
            return path.points[-1], len(path.points) - 1
        
        # Find index of lookahead point
        lookahead_index = current_index
        accumulated = 0.0
        
        while lookahead_index < len(path.points) - 1:
            p1 = path.points[lookahead_index]
            p2 = path.points[lookahead_index + 1]
            segment_length = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            
            if accumulated + segment_length >= lookahead:
                break
            
            accumulated += segment_length
            lookahead_index += 1
        
        return lookahead_point, lookahead_index
    
    def update_lookahead_distance(self, velocity: float):
        """Update lookahead distance based on velocity"""
        # Linear scaling: faster = look further ahead
        self.current_lookahead = min(
            self.lookahead_max,
            max(self.lookahead_min, self.lookahead_distance * (1.0 + velocity))
        )
    
    def compute_angular_velocity(self, v: float, curvature: float) -> float:
        """Convert curvature to angular velocity"""
        return v * curvature
    
    def is_goal_reached(self, robot_pose: Tuple[float, float, float],
                        goal_point: Tuple[float, float],
                        position_tolerance: float = 0.1,
                        heading_tolerance: float = math.radians(10)) -> Tuple[bool, bool]:
        """
        Check if goal is reached
        
        Returns:
            (position_reached, heading_reached)
        """
        x, y, heading = robot_pose
        gx, gy = goal_point
        
        # Check position
        distance = math.sqrt((gx - x)**2 + (gy - y)**2)
        position_reached = distance < position_tolerance
        
        # Check heading
        if position_reached:
            target_heading = math.atan2(gy - y, gx - x)
            heading_error = abs(self._normalize_angle(target_heading - heading))
            heading_reached = heading_error < heading_tolerance
        else:
            heading_reached = False
        
        return position_reached, heading_reached
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle