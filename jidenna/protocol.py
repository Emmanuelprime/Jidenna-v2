# protocol.py
import serial
import serial.tools.list_ports
from typing import Optional, Tuple
import logging
import time

logger = logging.getLogger(__name__)

class RobotProtocol:
    """Handles serial communication protocol with ESP32"""
    
    def __init__(self, port: str = None, baud_rate: int = 115200, timeout: float = 0.1):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_conn = None
        self._buffer = ""
    
    def connect(self) -> bool:
        """Establish serial connection"""
        try:
            if self.port is None:
                self.port = self._find_esp32_port()
                if self.port is None:
                    logger.error("No ESP32 device found")
                    return False
            
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            logger.info(f"Connected to ESP32 on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logger.info("Disconnected from ESP32")
    
    def is_connected(self) -> bool:
        """Check if serial connection is active"""
        return self.serial_conn is not None and self.serial_conn.is_open
    
    def send_velocity(self, v: float, w: float) -> bool:
        """Send velocity command to ESP32"""
        if not self.is_connected():
            return False
        
        try:
            # Format: V<linear_velocity>,<angular_velocity>\n
            command = f"V{v:.3f},{w:.3f}\n"
            self.serial_conn.write(command.encode('ascii'))
            self.serial_conn.flush()
            return True
        except Exception as e:
            logger.error(f"Failed to send velocity command: {e}")
            return False
    
    def send_command(self, command: str) -> bool:
        """
        Send raw command to ESP32
        
        Args:
            command: Raw command string (should end with newline)
            
        Returns:
            True if command was sent successfully
        """
        if not self.is_connected():
            logger.error("Not connected to ESP32")
            return False
        
        try:
            self.serial_conn.write(command.encode('ascii'))
            self.serial_conn.flush()
            logger.debug(f"Sent command: {command.strip()}")
            return True
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            return False
    
    def reset_odometry(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> bool:
        """
        Reset robot odometry on ESP32
        
        Args:
            x: New X position in meters
            y: New Y position in meters
            theta: New heading in radians
            
        Returns:
            True if reset command was sent successfully
        """
        # Format: R<x>,<y>,<theta>\n
        command = f"R{x:.3f},{y:.3f},{theta:.3f}\n"
        return self.send_command(command)
    
    def read_telemetry(self) -> Optional[Tuple[float, ...]]:
        """Read and parse telemetry data from ESP32"""
        if not self.is_connected():
            return None
        
        try:
            # Read available data
            if self.serial_conn.in_waiting > 0:
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                self._buffer += data.decode('ascii', errors='ignore')
                
                # Process complete lines
                while '\n' in self._buffer:
                    line, self._buffer = self._buffer.split('\n', 1)
                    line = line.strip()
                    
                    # Skip empty lines
                    if not line:
                        continue
                    
                    # Check for odometry reset acknowledgment
                    if line.startswith('ODOM_RESET'):
                        logger.info(f"Odometry reset acknowledged: {line}")
                        continue
                    
                    # Parse telemetry line
                    parsed = self._parse_telemetry_line(line)
                    if parsed:
                        return parsed
            
            return None
        except Exception as e:
            logger.error(f"Failed to read telemetry: {e}")
            return None
    
    def _parse_telemetry_line(self, line: str) -> Optional[Tuple[float, ...]]:
        """Parse a telemetry CSV line"""
        if not line:
            return None
        
        try:
            parts = line.split(',')
            if len(parts) != 8:
                # Check if it's a different response we should ignore
                if not line.startswith('ODOM_RESET'):
                    logger.debug(f"Malformed telemetry line: {line}")
                return None
            
            # Parse all fields as floats
            values = tuple(float(p) for p in parts)
            return values
        except ValueError:
            logger.debug(f"Failed to parse telemetry values: {line}")
            return None
    
    def _find_esp32_port(self) -> Optional[str]:
        """Auto-detect ESP32 serial port"""
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            # Look for common ESP32 identifiers
            if any(id in port.description for id in ['CP210', 'CH340', 'ESP32', 'USB Serial']):
                return port.device
        # If no specific match, return first available port
        if ports:
            return ports[0].device
        return None