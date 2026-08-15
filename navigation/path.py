import math
from typing import List, Tuple, Optional
import numpy as np

class Path:
    """Represents a 2D path for robot navigation"""
    
    def __init__(self, points: List[Tuple[float, float]] = None):
        self.points = points if points else []
        self._validate_path()
    
    def _validate_path(self):
        """Validate path points"""
        if len(self.points) < 2:
            raise ValueError("Path must contain at least 2 points")
        
        for point in self.points:
            if len(point) != 2:
                raise ValueError(f"Invalid point format: {point}")
            if not all(math.isfinite(p) for p in point):
                raise ValueError(f"Invalid point values: {point}")
    
    def add_point(self, point: Tuple[float, float]):
        """Add a point to the path"""
        self.points.append(point)
        self._validate_path()
    
    def get_points(self) -> List[Tuple[float, float]]:
        """Get all path points"""
        return self.points.copy()
    
    def get_length(self) -> float:
        """Calculate total path length"""
        length = 0.0
        for i in range(len(self.points) - 1):
            length += math.sqrt(
                (self.points[i+1][0] - self.points[i][0])**2 +
                (self.points[i+1][1] - self.points[i][1])**2
            )
        return length
    
    def get_nearest_point(self, position: Tuple[float, float]) -> Tuple[int, float]:
        """Find nearest point on path to given position
        
        Returns:
            (index, distance) - index of nearest path point and distance to it
        """
        if not self.points:
            raise ValueError("Path is empty")
        
        min_dist = float('inf')
        min_index = 0
        
        for i, point in enumerate(self.points):
            dist = math.sqrt(
                (point[0] - position[0])**2 + 
                (point[1] - position[1])**2
            )
            if dist < min_dist:
                min_dist = dist
                min_index = i
        
        return min_index, min_dist
    
    def get_point_at_distance(self, start_index: int, distance: float) -> Optional[Tuple[float, float]]:
        """Get point on path at specified distance from start_index"""
        if start_index < 0 or start_index >= len(self.points):
            return None
        
        if start_index == len(self.points) - 1:
            return self.points[-1]
        
        # Accumulate distance along path
        accumulated = 0.0
        current_index = start_index
        
        while current_index < len(self.points) - 1:
            p1 = self.points[current_index]
            p2 = self.points[current_index + 1]
            segment_length = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            
            if accumulated + segment_length >= distance:
                # Point is on this segment
                remaining = distance - accumulated
                if segment_length > 0:
                    t = remaining / segment_length
                    return (
                        p1[0] + t * (p2[0] - p1[0]),
                        p1[1] + t * (p2[1] - p1[1])
                    )
                else:
                    return p1
            
            accumulated += segment_length
            current_index += 1
        
        # Distance extends beyond path
        return self.points[-1]
    
    def get_remaining_distance(self, start_index: int) -> float:
        """Get remaining distance from start_index to end of path"""
        if start_index < 0 or start_index >= len(self.points):
            return 0.0
        
        distance = 0.0
        for i in range(start_index, len(self.points) - 1):
            p1 = self.points[i]
            p2 = self.points[i + 1]
            distance += math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        
        return distance