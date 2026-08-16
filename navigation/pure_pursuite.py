import math
from typing import Tuple, Optional, List
import numpy as np
from .path import Path

class PurePursuit:
    """Pure Pursuit path tracking algorithm for differential drive robots"""
    
    def __init__(self, lookahead_distance: float = 0.5, 
                 lookahead_min: float = 0.3,
                 lookahead_max: float = 1.5,
                 lookahead_speed_factor: float = 0.5,
                 max_curvature_rate: float = 2.0):
        """
        Initialize Pure Pursuit controller
        
        Args:
            lookahead_distance: Base lookahead distance in meters
            lookahead_min: Minimum lookahead distance
            lookahead_max: Maximum lookahead distance
            lookahead_speed_factor: How much to increase lookahead with speed
            max_curvature_rate: Maximum rate of curvature change (1/m²)
        """
        self.lookahead_distance = lookahead_distance
        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max
        self.lookahead_speed_factor = lookahead_speed_factor
        self.max_curvature_rate = max_curvature_rate
        self.current_lookahead = lookahead_distance
        
        # For curvature rate limiting
        self.last_curvature = 0.0
        self.last_time = None
        
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
        
        # Apply curvature rate limiting
        curvature = self._limit_curvature_rate(curvature)
        
        return curvature
    
    def select_lookahead_point(self, path: Path, robot_pose: Tuple[float, float, float],
                              current_index: int) -> Tuple[Optional[Tuple[float, float]], int]:
        """
        Select lookahead point on path with improved search
        
        Args:
            path: Path object
            robot_pose: (x, y, heading)
            current_index: Current nearest point index on path
        
        Returns:
            (lookahead_point, lookahead_index) - point and its index on path
        """
        # Check if we're near the end of the path
        if current_index >= len(path.points) - 2:
            # Near end, use last point
            return path.points[-1], len(path.points) - 1
        
        # Dynamic lookahead based on speed and path curvature
        lookahead = self.current_lookahead
        
        # Try to find point at lookahead distance
        lookahead_point = self._find_lookahead_point(path, robot_pose, current_index, lookahead)
        
        if lookahead_point is None:
            # Path too short, use last point
            return path.points[-1], len(path.points) - 1
        
        # Find index of lookahead point
        lookahead_index = self._find_lookahead_index(path, current_index, lookahead)
        
        return lookahead_point, lookahead_index
    
    def _find_lookahead_point(self, path: Path, robot_pose: Tuple[float, float, float],
                             start_index: int, lookahead_distance: float) -> Optional[Tuple[float, float]]:
        """
        Find lookahead point by searching forward from start_index
        
        Also checks if there are points closer to the robot within the lookahead circle
        """
        best_point = None
        best_distance = float('inf')
        
        # Search forward from start_index
        for i in range(start_index, len(path.points)):
            point = path.points[i]
            distance = math.sqrt(
                (point[0] - robot_pose[0])**2 + 
                (point[1] - robot_pose[1])**2
            )
            
            # Check if this point is at approximately the lookahead distance
            if abs(distance - lookahead_distance) < best_distance:
                best_distance = abs(distance - lookahead_distance)
                best_point = point
            
            # If we've gone too far, interpolate
            if distance > lookahead_distance and i > start_index:
                # Interpolate between previous point and this point
                prev_point = path.points[i - 1]
                prev_distance = math.sqrt(
                    (prev_point[0] - robot_pose[0])**2 + 
                    (prev_point[1] - robot_pose[1])**2
                )
                
                if prev_distance < lookahead_distance < distance:
                    # Interpolate
                    t = (lookahead_distance - prev_distance) / (distance - prev_distance)
                    interpolated = (
                        prev_point[0] + t * (point[0] - prev_point[0]),
                        prev_point[1] + t * (point[1] - prev_point[1])
                    )
                    return interpolated
        
        return best_point
    
    def _find_lookahead_index(self, path: Path, start_index: int, 
                             lookahead_distance: float) -> int:
        """Find the index of the lookahead point"""
        accumulated = 0.0
        current_index = start_index
        
        while current_index < len(path.points) - 1:
            p1 = path.points[current_index]
            p2 = path.points[current_index + 1]
            segment_length = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            
            if accumulated + segment_length >= lookahead_distance:
                break
            
            accumulated += segment_length
            current_index += 1
        
        return current_index
    
    def update_lookahead_distance(self, velocity: float, curvature: float = 0.0):
        """
        Update lookahead distance based on velocity and curvature
        
        Args:
            velocity: Current velocity (m/s)
            curvature: Current path curvature (1/m)
        """
        # Base lookahead increases with speed
        speed_lookahead = self.lookahead_distance * (1.0 + self.lookahead_speed_factor * abs(velocity))
        
        # Adjust based on curvature: shorter lookahead for sharper turns
        if abs(curvature) > 0.5:
            # Sharp turn: reduce lookahead to track curve better
            curvature_factor = 0.7
        elif abs(curvature) > 0.2:
            # Moderate turn: slight reduction
            curvature_factor = 0.85
        else:
            # Straight or gentle curve: full lookahead
            curvature_factor = 1.0
        
        # Apply limits
        self.current_lookahead = min(
            self.lookahead_max,
            max(self.lookahead_min, speed_lookahead * curvature_factor)
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
        
        # Check heading (only if position is reached)
        if position_reached:
            # If we're very close to goal, heading doesn't matter as much
            if distance < position_tolerance * 0.5:
                heading_reached = True  # Close enough
            else:
                target_heading = math.atan2(gy - y, gx - x)
                heading_error = abs(self._normalize_angle(target_heading - heading))
                heading_reached = heading_error < heading_tolerance
        else:
            heading_reached = False
        
        return position_reached, heading_reached
    
    def _limit_curvature_rate(self, curvature: float) -> float:
        """
        Limit rate of curvature change to prevent jerky steering
        
        Args:
            curvature: Desired curvature
        
        Returns:
            Rate-limited curvature
        """
        import time
        current_time = time.time()
        
        if self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0 and dt < 1.0:  # Sanity check
                # Calculate maximum change in curvature
                max_change = self.max_curvature_rate * dt
                
                # Limit change
                delta = curvature - self.last_curvature
                if abs(delta) > max_change:
                    curvature = self.last_curvature + math.copysign(max_change, delta)
        
        # Store for next iteration
        self.last_curvature = curvature
        self.last_time = current_time
        
        return curvature
    
    def reset(self):
        """Reset controller state"""
        self.current_lookahead = self.lookahead_distance
        self.last_curvature = 0.0
        self.last_time = None
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_debug_info(self) -> dict:
        """Get debug information about Pure Pursuit"""
        return {
            'current_lookahead': self.current_lookahead,
            'last_curvature': self.last_curvature,
            'lookahead_min': self.lookahead_min,
            'lookahead_max': self.lookahead_max
        }