import math

class TrajectoryGenerator:
    def __init__(self):
        pass
    
    def generate_line(self, start, end, num_points=20, relative=False):
        """Generate points along a straight line
        
        Args:
            start: Start point (x, y)
            end: End point (x, y)
            num_points: Number of points to generate
            relative: If True, generate relative movements instead of absolute positions
        """
        points = []
        
        if relative:
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            
            # Don't include the first point (0,0) for relative movements
            for i in range(1, num_points + 1):
                t = i / num_points
                x = dx * t
                y = dy * t
                points.append((x, y))
        else:
            for i in range(num_points):
                t = i / (num_points - 1)
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                points.append((x, y))
        
        return points
    
    def generate_square(self, side_length, start=(0, 0), num_points_per_side=10, relative=True):
        """Generate square path
        
        Args:
            side_length: Length of each side in meters
            start: Starting point (x, y)
            num_points_per_side: Points per side (if 1, just corners)
            relative: If True, generate relative movements (for PathFollower)
        """
        points = []
        
        if relative:
            movements = [
                (side_length, 0),      # Move forward
                (0, side_length),      # Move left
                (-side_length, 0),     # Move backward
                (0, -side_length)      # Move right
            ]
            
            for movement in movements:
                dx, dy = movement
                
                if num_points_per_side <= 1:
                    # Just the corner
                    points.append((dx, dy))
                else:
                    # Generate points along the side (excluding start point)
                    for i in range(1, num_points_per_side + 1):
                        t = i / num_points_per_side
                        x = dx * t
                        y = dy * t
                        points.append((x, y))
        else:
            x0, y0 = start
            
            # Side 1: forward
            points.extend(self.generate_line((x0, y0), (x0 + side_length, y0), num_points_per_side))
            # Side 2: left (skip first point to avoid duplicate)
            side2 = self.generate_line((x0 + side_length, y0), (x0 + side_length, y0 + side_length), num_points_per_side)
            points.extend(side2[1:])
            # Side 3: backward (skip first point)
            side3 = self.generate_line((x0 + side_length, y0 + side_length), (x0, y0 + side_length), num_points_per_side)
            points.extend(side3[1:])
            # Side 4: right (skip first point, skip last point to avoid duplicate with start)
            side4 = self.generate_line((x0, y0 + side_length), (x0, y0), num_points_per_side)
            points.extend(side4[1:-1])
        
        return points
    
    def generate_rectangle(self, length, width, start=(0, 0), num_points_per_side=10, relative=True):
        """Generate rectangle path
        
        Args:
            length: Length of rectangle (x direction)
            width: Width of rectangle (y direction)
            start: Starting point
            num_points_per_side: Points per side
            relative: If True, generate relative movements
        """
        points = []
        
        if relative:
            movements = [
                (length, 0),       # Move forward
                (0, width),        # Move left
                (-length, 0),      # Move backward
                (0, -width)        # Move right
            ]
            
            for movement in movements:
                dx, dy = movement
                
                if num_points_per_side <= 1:
                    points.append((dx, dy))
                else:
                    for i in range(1, num_points_per_side + 1):
                        t = i / num_points_per_side
                        x = dx * t
                        y = dy * t
                        points.append((x, y))
        else:
            x0, y0 = start
            points.extend(self.generate_line((x0, y0), (x0 + length, y0), num_points_per_side))
            side2 = self.generate_line((x0 + length, y0), (x0 + length, y0 + width), num_points_per_side)
            points.extend(side2[1:])
            side3 = self.generate_line((x0 + length, y0 + width), (x0, y0 + width), num_points_per_side)
            points.extend(side3[1:])
            side4 = self.generate_line((x0, y0 + width), (x0, y0), num_points_per_side)
            points.extend(side4[1:-1])
        
        return points
    
    def generate_circle(self, center, radius, num_points=36, relative=True):
        """Generate circular path
        
        Args:
            center: Center point (cx, cy)
            radius: Radius in meters
            num_points: Number of points
            relative: If True, generate relative movements (for PathFollower)
        """
        points = []
        
        if relative:
            # Generate relative movements for circle
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                next_angle = 2 * math.pi * (i + 1) / num_points
                
                current_x = radius * math.cos(angle)
                current_y = radius * math.sin(angle)
                next_x = radius * math.cos(next_angle)
                next_y = radius * math.sin(next_angle)
                
                dx = next_x - current_x
                dy = next_y - current_y
                points.append((dx, dy))
        else:
            cx, cy = center
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                x = cx + radius * math.cos(angle)
                y = cy + radius * math.sin(angle)
                points.append((x, y))
        
        return points
    
    def generate_custom_path(self, movements, num_points_per_segment=1):
        """Generate path from relative movements
        
        Args:
            movements: List of (dx, dy) movements
            num_points_per_segment: Points per segment (1 = just corners)
        
        Returns:
            List of waypoints (relative)
        """
        points = []
        
        for movement in movements:
            dx, dy = movement
            
            if num_points_per_segment <= 1:
                points.append((dx, dy))
            else:
                for i in range(1, num_points_per_segment + 1):
                    t = i / num_points_per_segment
                    x = dx * t
                    y = dy * t
                    points.append((x, y))
        
        return points