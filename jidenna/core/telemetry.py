"""
Telemetry data management with improved parsing
"""

import time
from dataclasses import dataclass
from typing import Optional, Dict, Any
import threading
import logging
import re

logger = logging.getLogger(__name__)


@dataclass
class TelemetryData:
    """Telemetry data structure"""
    timestamp: float
    left_speed: float
    right_speed: float
    linear_velocity: float
    angular_velocity: float
    filtered_angular_velocity: float
    yaw: float
    x: float
    y: float
    left_pwm: int
    right_pwm: int
    state: int
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        return {
            'timestamp': self.timestamp,
            'left_speed': self.left_speed,
            'right_speed': self.right_speed,
            'linear_velocity': self.linear_velocity,
            'angular_velocity': self.angular_velocity,
            'filtered_angular_velocity': self.filtered_angular_velocity,
            'yaw': self.yaw,
            'x': self.x,
            'y': self.y,
            'left_pwm': self.left_pwm,
            'right_pwm': self.right_pwm,
            'state': self.state
        }


class TelemetryManager:
    """Manages telemetry data and history"""
    
    def __init__(self, max_history: int = 1000):
        self._latest: Optional[TelemetryData] = None
        self._history: list = []
        self._max_history = max_history
        self._lock = threading.RLock()
        self._callbacks = []
        self._last_valid = 0
        
    def parse_telemetry(self, line: str) -> Optional[TelemetryData]:
        """Parse telemetry line from robot"""
        try:
            if not line or not line.startswith("CNT"):
                return None
            
            # Remove "CNT" prefix if present
            if line.startswith("CNT,"):
                line = line[4:]
            
            parts = line.split(',')
            
            # Log the raw data for debugging
            logger.debug(f"Parsing telemetry: {parts}")
            
            # Check if we have enough parts
            if len(parts) < 12:
                logger.warning(f"Incomplete telemetry data: {len(parts)} fields")
                return None
            
            # Parse each field with error handling
            try:
                data = TelemetryData(
                    timestamp=float(parts[0].strip()) if parts[0].strip() else 0.0,
                    left_speed=float(parts[1].strip()) if len(parts) > 1 and parts[1].strip() else 0.0,
                    right_speed=float(parts[2].strip()) if len(parts) > 2 and parts[2].strip() else 0.0,
                    linear_velocity=float(parts[3].strip()) if len(parts) > 3 and parts[3].strip() else 0.0,
                    angular_velocity=float(parts[4].strip()) if len(parts) > 4 and parts[4].strip() else 0.0,
                    filtered_angular_velocity=float(parts[5].strip()) if len(parts) > 5 and parts[5].strip() else 0.0,
                    yaw=float(parts[6].strip()) if len(parts) > 6 and parts[6].strip() else 0.0,
                    x=float(parts[7].strip()) if len(parts) > 7 and parts[7].strip() else 0.0,
                    y=float(parts[8].strip()) if len(parts) > 8 and parts[8].strip() else 0.0,
                    left_pwm=int(float(parts[9].strip())) if len(parts) > 9 and parts[9].strip() else 0,
                    right_pwm=int(float(parts[10].strip())) if len(parts) > 10 and parts[10].strip() else 0,
                    state=int(float(parts[11].strip())) if len(parts) > 11 and parts[11].strip() else 0
                )
            except (ValueError, IndexError) as e:
                logger.error(f"Error parsing telemetry values: {e}")
                return None
            
            with self._lock:
                self._latest = data
                self._history.append(data)
                if len(self._history) > self._max_history:
                    self._history.pop(0)
                self._last_valid = time.time()
            
            return data
            
        except Exception as e:
            logger.error(f"Error parsing telemetry: {e}")
            return None
    
    def get_latest(self) -> Optional[TelemetryData]:
        """Get latest telemetry data"""
        with self._lock:
            return self._latest
    
    def get_history(self, count: int = 100) -> list:
        """Get telemetry history"""
        with self._lock:
            return self._history[-count:] if self._history else []
    
    def clear_history(self):
        """Clear telemetry history"""
        with self._lock:
            self._history.clear()
    
    def wait_for_data(self, timeout: float = 5.0) -> bool:
        """Wait for first telemetry data"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self._lock:
                if self._latest is not None:
                    return True
            time.sleep(0.1)
        return False
    
    def has_data(self) -> bool:
        """Check if we have received any telemetry data"""
        with self._lock:
            return self._latest is not None
    
    def get_data_age(self) -> float:
        """Get age of latest data in seconds"""
        with self._lock:
            if self._latest is None:
                return float('inf')
            return time.time() - self._last_valid