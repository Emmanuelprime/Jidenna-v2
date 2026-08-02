"""
Obstacle avoidance plugin using simulated sensors
"""

import math
import time
from typing import List, Dict, Optional
from enum import Enum

from .base_plugin import BasePlugin
import logging

logger = logging.getLogger(__name__)


class ObstacleType(Enum):
    """Types of obstacles"""
    WALL = "wall"
    OBJECT = "object"
    PERSON = "person"
    UNKNOWN = "unknown"


class ObstacleAvoidancePlugin(BasePlugin):
    """
    Obstacle avoidance plugin
    
    This is a simulation plugin. Replace with actual sensors.
    """
    
    def __init__(self, name: str = "ObstacleAvoidancePlugin"):
        super().__init__(name)
        self.active = False
        self.min_distance = 0.3  # meters
        self.avoidance_distance = 0.5  # meters
        self.sensor_range = 2.0  # meters
        
        # Simulated obstacles (replace with real sensors)
        self.obstacles: List[Dict] = []
        self._scan_results = []
        
    def initialize(self):
        super().initialize()
        self.active = True
        logger.info("Obstacle avoidance initialized")
    
    def shutdown(self):
        self.active = False
        super().shutdown()
    
    def requires_thread(self) -> bool:
        return True
    
    def run(self):
        """Obstacle avoidance thread"""
        while self._enabled and self.active:
            self._scan()
            
            # Check for obstacles in path
            if self._obstacle_in_path():
                self._avoid_obstacle()
            
            time.sleep(0.1)
    
    def _scan(self):
        """Simulated scanning (replace with actual sensors)"""
        # In reality, read from sensors like ultrasonic, LIDAR, etc.
        self._scan_results = []
        
        # Simulate obstacles (for testing)
        telemetry = self.controller.get_telemetry()
        if telemetry:
            # Check if any obstacles near current position
            for obs in self.obstacles:
                dx = obs['x'] - telemetry.x
                dy = obs['y'] - telemetry.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < self.sensor_range:
                    angle = math.atan2(dy, dx)
                    self._scan_results.append({
                        'distance': distance,
                        'angle': angle,
                        'type': obs.get('type', ObstacleType.UNKNOWN)
                    })
    
    def _obstacle_in_path(self) -> bool:
        """Check if obstacle is in robot's path"""
        if not self._scan_results:
            return False
        
        # Check for obstacles ahead
        telemetry = self.controller.get_telemetry()
        if not telemetry:
            return False
        
        yaw_rad = telemetry.yaw * math.pi / 180.0
        
        for obs in self._scan_results:
            # Check if obstacle is in front
            angle_diff = obs['angle'] - yaw_rad
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            if abs(angle_diff) < math.pi/4 and obs['distance'] < self.min_distance:
                return True
        
        return False
    
    def _avoid_obstacle(self):
        """Perform obstacle avoidance"""
        if not self._scan_results:
            return
        
        # Find closest obstacle
        closest = min(self._scan_results, key=lambda x: x['distance'])
        
        # Decide which way to turn
        angle_to_obstacle = closest['angle']
        telemetry = self.controller.get_telemetry()
        yaw_rad = telemetry.yaw * math.pi / 180.0
        angle_diff = angle_to_obstacle - yaw_rad
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Turn away from obstacle
        turn_direction = 1 if angle_diff > 0 else -1
        
        logger.info(f"Avoiding obstacle at {closest['distance']:.2f}m")
        
        # Execute avoidance maneuver
        self.controller.stop()
        time.sleep(0.1)
        self.controller.turn_left(turn_direction * 0.5, duration=0.5)
        time.sleep(0.1)
        self.controller.move_forward(0.2, duration=0.5)
        time.sleep(0.1)
        
        # Resume normal operation (will be handled by navigation plugin)
    
    def add_obstacle(self, x: float, y: float, obs_type: ObstacleType = ObstacleType.UNKNOWN):
        """Add a simulated obstacle"""
        self.obstacles.append({
            'x': x,
            'y': y,
            'type': obs_type
        })
    
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()