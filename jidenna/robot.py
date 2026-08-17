# robot.py
from typing import Optional
import logging
import threading
import time
import math
from .protocol import RobotProtocol
from .state import RobotState

logger = logging.getLogger(__name__)

class RobotAPI:
    """High-level robot communication interface"""
    
    def __init__(self, port: str = None, baud_rate: int = 115200):
        self.protocol = RobotProtocol(port, baud_rate)
        self.state = RobotState()
        self._running = False
        self._telemetry_thread = None
        self._lock = threading.Lock()
        self._command_timeout = 2.0  # seconds (matches ESP32 timeout)
        self._last_command_time = 0
    
    def connect(self) -> bool:
        """Connect to robot and start telemetry reading"""
        if self.protocol.connect():
            self._running = True
            self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
            self._telemetry_thread.start()
            return True
        return False
    
    def disconnect(self):
        """Stop robot and disconnect"""
        self.stop()  # Send stop command
        time.sleep(0.1)  # Brief delay to ensure command is sent
        self._running = False
        if self._telemetry_thread:
            self._telemetry_thread.join(timeout=1.0)
        self.protocol.disconnect()
    
    def is_connected(self) -> bool:
        """Check if connected to robot"""
        return self.protocol.is_connected()
    
    def set_velocity(self, v: float, w: float) -> bool:
        """Set robot velocity (linear v in m/s, angular w in rad/s)"""
        with self._lock:
            success = self.protocol.send_velocity(v, w)
            if success:
                self._last_command_time = time.time()
            return success
    
    def stop(self) -> bool:
        """Stop robot immediately"""
        return self.set_velocity(0.0, 0.0)
    
    def reset_odometry(self, x: float = 0.0, y: float = 0.0, heading: float = 0.0) -> bool:
        """
        Reset robot odometry on the ESP32
        
        Args:
            x: New X position in meters
            y: New Y position in meters
            heading: New heading in radians
            
        Returns:
            True if reset command was sent successfully
        """
        with self._lock:
            # Send reset command via protocol
            success = self.protocol.reset_odometry(x, y, heading)
            
            if success:
                # Update local state immediately to avoid mismatch
                self.state.x = x
                self.state.y = y
                self.state.heading = heading
                self._last_command_time = time.time()
                
                heading_deg = math.degrees(heading)
                logger.info(f"Odometry reset to: ({x:.3f}, {y:.3f}, {heading_deg:.1f}°)")
            
            return success
    
    def get_state(self) -> RobotState:
        """Get current robot state"""
        with self._lock:
            return RobotState(
                x=self.state.x,
                y=self.state.y,
                heading=self.state.heading,
                left_velocity=self.state.left_velocity,
                right_velocity=self.state.right_velocity,
                imu_angle_z=self.state.imu_angle_z,
                imu_gyro_z=self.state.imu_gyro_z,
                timestamp=self.state.timestamp
            )
    
    def _telemetry_loop(self):
        """Background thread for reading telemetry"""
        while self._running:
            telemetry = self.protocol.read_telemetry()
            if telemetry:
                with self._lock:
                    self.state.x = telemetry[0]
                    self.state.y = telemetry[1]
                    self.state.heading = telemetry[2]
                    self.state.left_velocity = telemetry[3]
                    self.state.right_velocity = telemetry[4]
                    self.state.imu_angle_z = telemetry[5]
                    self.state.imu_gyro_z = telemetry[6]
                    self.state.timestamp = int(telemetry[7])
            else:
                time.sleep(0.01)  # Small sleep if no data
    
    def check_communication_timeout(self) -> bool:
        """Check if communication with robot has timed out"""
        if time.time() - self._last_command_time > self._command_timeout:
            return True
        return False