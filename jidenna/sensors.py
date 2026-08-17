#!/usr/bin/env python3
"""
Ultrasonic sensor API for data collection only
Reads from second ESP32 with ultrasonic sensors
ESP32 sends distances in METERS (e.g., 1.653, 0.560, 0.427)
"""

import serial
import serial.tools.list_ports
import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional, List, Callable
import math

logger = logging.getLogger(__name__)

@dataclass
class UltrasonicData:
    """Data from ultrasonic sensors (all distances in meters)"""
    timestamp: int = 0  # milliseconds
    left: float = -1.0  # meters (-1 = no reading/timeout)
    center: float = -1.0  # meters
    right: float = -1.0  # meters
    
    @property
    def left_cm(self) -> float:
        """Left distance in centimeters"""
        return self.left * 100.0 if self.left > 0 else -1.0
    
    @property
    def center_cm(self) -> float:
        """Center distance in centimeters"""
        return self.center * 100.0 if self.center > 0 else -1.0
    
    @property
    def right_cm(self) -> float:
        """Right distance in centimeters"""
        return self.right * 100.0 if self.right > 0 else -1.0
    
    @property
    def min_distance(self) -> float:
        """Minimum distance across all sensors (meters)"""
        valid = [d for d in [self.left, self.center, self.right] if d > 0]
        return min(valid) if valid else -1.0
    
    @property
    def min_distance_cm(self) -> float:
        """Minimum distance in centimeters"""
        min_m = self.min_distance
        return min_m * 100.0 if min_m > 0 else -1.0
    
    @property
    def max_distance(self) -> float:
        """Maximum distance across all sensors (meters)"""
        valid = [d for d in [self.left, self.center, self.right] if d > 0]
        return max(valid) if valid else -1.0
    
    @property
    def all_valid_distances(self) -> List[float]:
        """Get list of valid distances (meters)"""
        return [d for d in [self.left, self.center, self.right] if d > 0]
    
    @property
    def num_valid_readings(self) -> int:
        """Number of valid sensor readings"""
        return len(self.all_valid_distances)
    
    def is_valid(self) -> bool:
        """Check if data timestamp is valid"""
        return self.timestamp > 0
    
    def has_any_reading(self) -> bool:
        """Check if any sensor has a valid reading"""
        return self.num_valid_readings > 0
    
    def get_sensor_reading(self, sensor: str) -> float:
        """
        Get reading from specific sensor
        
        Args:
            sensor: 'left', 'center', or 'right'
        
        Returns:
            Distance in meters, or -1 if invalid
        """
        if sensor.lower() == 'left':
            return self.left
        elif sensor.lower() == 'center':
            return self.center
        elif sensor.lower() == 'right':
            return self.right
        else:
            raise ValueError(f"Unknown sensor: {sensor}")
    
    def get_sensor_reading_cm(self, sensor: str) -> float:
        """
        Get reading from specific sensor in centimeters
        
        Args:
            sensor: 'left', 'center', or 'right'
        
        Returns:
            Distance in cm, or -1 if invalid
        """
        reading_m = self.get_sensor_reading(sensor)
        return reading_m * 100.0 if reading_m > 0 else -1.0
    
    def to_dict(self) -> dict:
        """Convert to dictionary (all distances in meters)"""
        return {
            'timestamp': self.timestamp,
            'left_m': self.left,
            'center_m': self.center,
            'right_m': self.right,
            'left_cm': self.left_cm,
            'center_cm': self.center_cm,
            'right_cm': self.right_cm,
            'min_distance_m': self.min_distance,
            'min_distance_cm': self.min_distance_cm,
            'num_valid': self.num_valid_readings
        }
    
    def __str__(self) -> str:
        """String representation (in meters)"""
        return (f"UltrasonicData(ts={self.timestamp}, "
                f"L={self.left:.3f}m, C={self.center:.3f}m, R={self.right:.3f}m)")


class UltrasonicAPI:
    """Communication API for ultrasonic sensor ESP32 (data collection only)"""
    
    def __init__(self, port: str = None, baud_rate: int = 115200, 
                 timeout: float = 0.5, auto_start: bool = True):
        """
        Initialize ultrasonic sensor API
        
        Args:
            port: Serial port for ultrasonic ESP32
            baud_rate: Baud rate (should match ESP32)
            timeout: Serial timeout in seconds
            auto_start: Start reading thread automatically on connect
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.auto_start = auto_start
        self.serial_conn = None
        self._buffer = ""
        self._running = False
        self._read_thread = None
        self._lock = threading.Lock()
        
        # Latest data (stored in meters)
        self.latest_data = UltrasonicData()
        
        # Statistics
        self.total_readings = 0
        self.valid_readings = 0
        self.invalid_lines = 0
        self.parse_errors = 0
        self.last_read_time = 0
        
        # Data callback
        self.data_callback: Optional[Callable[[UltrasonicData], None]] = None
    
    def connect(self) -> bool:
        """Connect to ultrasonic sensor ESP32"""
        try:
            if self.port is None:
                self.port = self._find_ultrasonic_port()
                if self.port is None:
                    logger.error("No ultrasonic sensor ESP32 found")
                    return False
            
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            
            # Flush any old data
            self.serial_conn.reset_input_buffer()
            time.sleep(0.1)
            
            logger.info(f"Connected to ultrasonic sensors on {self.port}")
            
            # Start reading thread if auto_start
            if self.auto_start:
                self.start_reading()
            
            return True
        except serial.SerialException as e:
            logger.error(f"Serial error connecting to {self.port}: {e}")
            return False
        except Exception as e:
            logger.error(f"Failed to connect to ultrasonic sensors: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from ultrasonic sensor ESP32"""
        self.stop_reading()
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logger.info("Disconnected from ultrasonic sensors")
    
    def is_connected(self) -> bool:
        """Check if connected"""
        return self.serial_conn is not None and self.serial_conn.is_open
    
    def start_reading(self):
        """Start background reading thread"""
        if not self.is_connected():
            logger.error("Not connected")
            return
        
        if self._running:
            logger.warning("Already reading")
            return
        
        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        logger.debug("Started ultrasonic reading thread")
    
    def stop_reading(self):
        """Stop background reading thread"""
        self._running = False
        if self._read_thread:
            self._read_thread.join(timeout=1.0)
            self._read_thread = None
    
    def get_data(self) -> UltrasonicData:
        """Get latest ultrasonic data in meters (thread-safe)"""
        with self._lock:
            return UltrasonicData(
                timestamp=self.latest_data.timestamp,
                left=self.latest_data.left,
                center=self.latest_data.center,
                right=self.latest_data.right
            )
    
    def wait_for_data(self, timeout: float = 2.0) -> Optional[UltrasonicData]:
        """
        Wait for valid data
        
        Args:
            timeout: Maximum time to wait in seconds
        
        Returns:
            UltrasonicData if valid data received, None if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            data = self.get_data()
            if data.is_valid() and data.has_any_reading():
                return data
            time.sleep(0.05)
        return None
    
    def set_data_callback(self, callback: Callable[[UltrasonicData], None]):
        """Set callback for new data"""
        self.data_callback = callback
    
    def clear_data_callback(self):
        """Clear data callback"""
        self.data_callback = None
    
    def get_statistics(self) -> dict:
        """Get reading statistics"""
        with self._lock:
            total = self.total_readings
            valid = self.valid_readings
        
        return {
            'total_readings': total,
            'valid_readings': valid,
            'invalid_lines': self.invalid_lines,
            'parse_errors': self.parse_errors,
            'validity_rate': (valid / total * 100) if total > 0 else 0,
            'last_read_time': self.last_read_time,
            'is_reading': self._running,
            'is_connected': self.is_connected()
        }
    
    def _read_loop(self):
        """Background thread for reading serial data"""
        while self._running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    # Read available data
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    self._buffer += data.decode('ascii', errors='ignore')
                    
                    # Process complete lines
                    while '\n' in self._buffer:
                        line, self._buffer = self._buffer.split('\n', 1)
                        line = line.strip()
                        
                        # Skip empty lines
                        if not line:
                            continue
                        
                        # Only process ULTRA lines
                        if not line.startswith('ULTRA'):
                            self.invalid_lines += 1
                            logger.debug(f"Skipping non-ULTRA line: {line[:50]}")
                            continue
                        
                        parsed = self._parse_line(line)
                        
                        if parsed:
                            with self._lock:
                                self.latest_data = parsed
                                self.total_readings += 1
                                if parsed.has_any_reading():
                                    self.valid_readings += 1
                                self.last_read_time = time.time()
                            
                            # Call callback if set
                            if self.data_callback:
                                try:
                                    self.data_callback(parsed)
                                except Exception as e:
                                    logger.error(f"Data callback error: {e}")
                        else:
                            self.parse_errors += 1
                else:
                    time.sleep(0.01)  # Wait for data
            except serial.SerialException as e:
                logger.error(f"Serial read error: {e}")
                self._running = False
                break
            except Exception as e:
                logger.error(f"Error reading ultrasonic data: {e}")
                time.sleep(0.1)
    
    def _parse_line(self, line: str) -> Optional[UltrasonicData]:
        """
        Parse a line from ultrasonic ESP32
        ESP32 sends distances in METERS (e.g., 1.653, 0.560, 0.427)
        """
        if not line or not line.startswith('ULTRA'):
            return None
        
        try:
            parts = line.split(',')
            if len(parts) != 5:
                logger.debug(f"Malformed ultrasonic line (expected 5 parts, got {len(parts)}): {line}")
                return None
            
            # Parse values (ESP32 sends METERS)
            timestamp = int(parts[1].strip())
            left_m = float(parts[2].strip())
            center_m = float(parts[3].strip())
            right_m = float(parts[4].strip())
            
            # Validate values
            if timestamp < 0:
                logger.debug(f"Invalid timestamp: {timestamp}")
                return None
            
            # Distances should be between -1 and 5 meters
            for val, name in [(left_m, 'left'), (center_m, 'center'), (right_m, 'right')]:
                if val < -1 or val > 5.0:
                    logger.debug(f"Invalid {name} distance: {val}m")
                    return None
            
            # Use values directly (already in meters)
            # -1 means no reading/timeout
            return UltrasonicData(
                timestamp=timestamp,
                left=left_m if left_m > 0 else -1.0,
                center=center_m if center_m > 0 else -1.0,
                right=right_m if right_m > 0 else -1.0
            )
        except (ValueError, IndexError) as e:
            logger.debug(f"Failed to parse ultrasonic data: {e}")
            return None
    
    def _find_ultrasonic_port(self) -> Optional[str]:
        """Auto-detect ultrasonic sensor ESP32 port"""
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            # Look for common ESP32 identifiers
            if any(id in port.description for id in ['CP210', 'CH340', 'ESP32', 'USB Serial']):
                return port.device
        return None


class SensorManager:
    """
    Manages multiple sensors (ultrasonic, future GPS, etc.)
    Provides unified access to all sensor data
    """
    
    def __init__(self):
        """Initialize sensor manager"""
        self.ultrasonic: Optional[UltrasonicAPI] = None
        self.gps = None  # Future GPS
        self.lidar = None  # Future LiDAR
    
    def add_ultrasonic(self, port: str = None, baud_rate: int = 115200) -> UltrasonicAPI:
        """Add ultrasonic sensor"""
        self.ultrasonic = UltrasonicAPI(port=port, baud_rate=baud_rate)
        return self.ultrasonic
    
    def get_ultrasonic_data(self) -> Optional[UltrasonicData]:
        """Get ultrasonic data if available"""
        if self.ultrasonic and self.ultrasonic.is_connected():
            return self.ultrasonic.get_data()
        return None
    
    def connect_all(self) -> bool:
        """Connect all sensors"""
        success = True
        
        if self.ultrasonic:
            if not self.ultrasonic.connect():
                logger.error("Failed to connect ultrasonic sensors")
                success = False
        
        # Future: connect GPS, LiDAR, etc.
        
        return success
    
    def disconnect_all(self):
        """Disconnect all sensors"""
        if self.ultrasonic:
            self.ultrasonic.disconnect()
        
        # Future: disconnect GPS, LiDAR, etc.
    
    def get_all_data(self) -> dict:
        """Get data from all sensors"""
        data = {}
        
        if self.ultrasonic and self.ultrasonic.is_connected():
            ultrasonic_data = self.ultrasonic.get_data()
            data['ultrasonic'] = ultrasonic_data.to_dict() if ultrasonic_data.is_valid() else None
        
        # Future: add GPS, LiDAR data
        
        return data