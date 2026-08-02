"""
Example of creating a custom plugin
"""

from plugins.base_plugin import BasePlugin
from core.robot_controller import RobotController
import time
import logging

logger = logging.getLogger(__name__)


class MyCustomPlugin(BasePlugin):
    """Custom plugin example"""
    
    def __init__(self, name: str = "MyCustomPlugin"):
        super().__init__(name)
        self.counter = 0
        
    def initialize(self):
        super().initialize()
        logger.info("Custom plugin initialized!")
        
    def shutdown(self):
        logger.info("Custom plugin shutting down...")
        super().shutdown()
    
    def requires_thread(self) -> bool:
        return True
    
    def run(self):
        """Main plugin loop"""
        while self._enabled:
            self.counter += 1
            if self.counter % 10 == 0:
                logger.info(f"Plugin running: {self.counter}")
            
            # Do something with robot
            if self.controller and self.counter % 50 == 0:
                telemetry = self.controller.get_telemetry()
                if telemetry:
                    logger.info(f"Current position: ({telemetry.x:.3f}, {telemetry.y:.3f})")
            
            time.sleep(0.1)
    
    def on_telemetry(self, data):
        """Handle telemetry data"""
        # React to telemetry data
        if data.linear_velocity > 0.5:
            logger.info("Robot moving fast!")


def main():
    with RobotController(port="COM19") as robot:
        # Register custom plugin
        plugin = MyCustomPlugin()
        robot.register_plugin(plugin)
        
        # Move robot and see plugin output
        robot.move_forward(0.3, duration=3.0)
        time.sleep(2)
        robot.turn_degrees(90)
        time.sleep(2)
        robot.stop()

if __name__ == "__main__":
    main()