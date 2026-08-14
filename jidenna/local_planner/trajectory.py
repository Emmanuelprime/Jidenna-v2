import math

class TrajectoryGenerator:
    def __init__(self):
        pass
    
    def generate_line(self, start, end, num_points=20):
        """Generate points along a straight line"""
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            points.append((x, y))
        return points
    
    def generate_square(self, side_length, start=(0, 0), num_points_per_side=10):
        """Generate square path"""
        points = []
        x0, y0 = start
        
        # Side 1: forward
        points.extend(self.generate_line((x0, y0), (x0 + side_length, y0), num_points_per_side))
        # Side 2: left
        points.extend(self.generate_line((x0 + side_length, y0), (x0 + side_length, y0 + side_length), num_points_per_side))
        # Side 3: backward
        points.extend(self.generate_line((x0 + side_length, y0 + side_length), (x0, y0 + side_length), num_points_per_side))
        # Side 4: right
        points.extend(self.generate_line((x0, y0 + side_length), (x0, y0), num_points_per_side))
        
        return points
    
    def generate_circle(self, center, radius, num_points=36):
        """Generate circular path"""
        points = []
        cx, cy = center
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            points.append((x, y))
        return points