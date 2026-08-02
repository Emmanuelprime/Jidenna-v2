"""
Navigation plugin - Safer version
"""

import math
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass
import threading
import logging

from .base_plugin import BasePlugin

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    x: float
    y: float
    tolerance: float = 0.05
    speed: float = 0.3
    heading: Optional[float] = None


class NavigationPlugin(BasePlugin):
    """Navigation plugin with waypoint following"""
    
    def __init__(self, name: str = "NavigationPlugin"):
        super().__init__(name)
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = 0
        self.navigation_active = False
        self.nav_thread: Optional[threading.Thread] = None
        self.speed = 0.3
        self.tolerance = 0.05
        
    def initialize(self):
        super().initialize()
        self.navigation_active = True
        logger.info("Navigation plugin initialized")
        
    def shutdown(self):
        self.navigation_active = False
        if self.nav_thread and self.nav_thread.is_alive():
            self.nav_thread.join(timeout=1.0)
        super().shutdown()
    
    def requires_thread(self) -> bool:
        return True
    
    def run(self):
        """Navigation thread"""
        while self._enabled and self.navigation_active:
            try:
                if self.waypoints and self.controller:
                    self._follow_waypoints()
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Error in navigation thread: {e}")
                time.sleep(0.5)
    
    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """Set waypoints for navigation"""
        self.waypoints = [Waypoint(x=x, y=y) for x, y in waypoints]
        self.current_waypoint_index = 0
        logger.info(f"Set {len(self.waypoints)} waypoints")
    
    def add_waypoint(self, x: float, y: float, speed: float = 0.3):
        self.waypoints.append(Waypoint(x=x, y=y, speed=speed))
    
    def _follow_waypoints(self):
        """Follow waypoints sequentially"""
        if self.current_waypoint_index >= len(self.waypoints):
            return
        
        waypoint = self.waypoints[self.current_waypoint_index]
        
        telemetry = self.controller.get_telemetry()
        if not telemetry:
            return
        
        dx = waypoint.x - telemetry.x
        dy = waypoint.y - telemetry.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < waypoint.tolerance:
            logger.info(f"Reached waypoint {self.current_waypoint_index}")
            self.current_waypoint_index += 1
            self.controller.stop()
            return
        
        target_angle = math.atan2(dy, dx)
        current_angle = telemetry.yaw * math.pi / 180.0
        
        angle_diff = target_angle - current_angle
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        linear = min(self.speed, distance * 2)
        angular = max(-0.5, min(0.5, angle_diff * 2))
        
        self.controller.send_velocity(linear, angular)
    
    def stop(self):
        """Stop navigation"""
        self.waypoints.clear()
        self.current_waypoint_index = 0
        if self.controller:
            self.controller.stop()