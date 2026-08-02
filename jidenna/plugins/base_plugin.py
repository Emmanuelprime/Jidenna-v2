"""
Base plugin class for robot controller plugins
"""

from abc import ABC, abstractmethod
from typing import Optional, Any
import logging
import time

logger = logging.getLogger(__name__)


class BasePlugin(ABC):
    """
    Base class for all plugins
    
    Implement this to create custom plugins for the robot controller.
    """
    
    def __init__(self, name: str):
        self.name = name
        self.controller = None
        self._enabled = True
        self._initialized = False
        
    def get_name(self) -> str:
        """Get plugin name"""
        return self.name
    
    def set_controller(self, controller):
        """Set the controller instance"""
        self.controller = controller
    
    @abstractmethod
    def initialize(self):
        """Initialize the plugin"""
        self._initialized = True
        logger.info(f"Plugin {self.name} initialized")
    
    @abstractmethod
    def shutdown(self):
        """Shutdown the plugin"""
        self._enabled = False
        logger.info(f"Plugin {self.name} shut down")
    
    def requires_thread(self) -> bool:
        """Does this plugin require a background thread?"""
        return False
    
    def run(self):
        """Main plugin loop (if thread is required)"""
        while self._enabled:
            self.update()
            time.sleep(0.01)
    
    def update(self):
        """Update method called periodically"""
        pass
    
    def on_telemetry(self, data):
        """Called when new telemetry data is received"""
        pass
    
    def on_state_change(self, state):
        """Called when robot state changes"""
        pass
    
    def on_error(self, error):
        """Called when an error occurs"""
        pass
    
    def enable(self):
        """Enable the plugin"""
        self._enabled = True
        
    def disable(self):
        """Disable the plugin"""
        self._enabled = False