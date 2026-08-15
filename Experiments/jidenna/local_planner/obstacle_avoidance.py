import math
import time

class ObstacleAvoidance:
    def __init__(self, robot, heading_controller, safe_distance=0.3):
        self.robot = robot
        self.heading_controller = heading_controller
        self.safe_distance = safe_distance
        self.obstacle_detected = False
        
    def check_obstacles(self, sensor_readings):
        """Check if any obstacle is within safe distance"""
        if not sensor_readings:
            return False
        
        for distance in sensor_readings:
            if distance < self.safe_distance:
                return True
        return False
    
    def avoid_obstacle(self, sensor_readings):
        """Simple avoidance - turn away from obstacle"""
        if not sensor_readings:
            return False
        
        # Find closest obstacle direction
        min_distance = min(sensor_readings)
        min_index = sensor_readings.index(min_distance)
        
        # Turn away from obstacle
        if min_index < len(sensor_readings) / 2:
            # Obstacle on left, turn right
            turn_direction = -1
        else:
            # Obstacle on right, turn left
            turn_direction = 1
        
        print(f"Avoiding obstacle at {min_distance:.2f}m")
        
        # Turn away
        self.robot.drive(0, turn_direction * 0.5)
        time.sleep(1.5)
        self.robot.stop()
        
        # Move forward
        self.robot.drive(0.2, 0)
        time.sleep(1.0)
        self.robot.stop()
        
        return True