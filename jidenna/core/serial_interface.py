"""
Serial communication interface with automatic reconnection
"""

import serial
import serial.tools.list_ports
import time
import threading
from typing import Optional, List
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class SerialConfig:
    """Serial port configuration"""
    port: str
    baud: int = 115200
    timeout: float = 0.1
    write_timeout: float = 1.0
    reconnect_attempts: int = 3
    reconnect_delay: float = 1.0


class SerialInterface:
    """Robust serial communication interface"""
    
    def __init__(self, config: SerialConfig):
        self.config = config
        self.serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._connected = False
        self._buffer = ""
        
    def connect(self) -> bool:
        """Connect to serial port"""
        with self._lock:
            if self._connected:
                return True
            
            try:
                self.serial = serial.Serial(
                    port=self.config.port,
                    baudrate=self.config.baud,
                    timeout=self.config.timeout,
                    write_timeout=self.config.write_timeout
                )
                self.serial.flushInput()
                self.serial.flushOutput()
                self._connected = True
                logger.info(f"Connected to {self.config.port}")
                return True
                
            except serial.SerialException as e:
                logger.error(f"Failed to connect to {self.config.port}: {e}")
                return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        with self._lock:
            if self.serial and self.serial.is_open:
                try:
                    self.serial.close()
                except Exception:
                    pass
            self._connected = False
            self.serial = None
    
    def is_connected(self) -> bool:
        """Check if connected"""
        with self._lock:
            return self._connected and self.serial and self.serial.is_open
    
    def write(self, data: str) -> bool:
        """Write data to serial port"""
        if not self.is_connected():
            return False
        
        try:
            with self._lock:
                self.serial.write(data.encode())
                self.serial.flush()
            return True
        except serial.SerialException as e:
            logger.error(f"Write error: {e}")
            self._connected = False
            return False
    
    def readline(self) -> Optional[str]:
        """Read a line from serial port"""
        if not self.is_connected():
            return None
        
        try:
            with self._lock:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    return line
            return None
        except serial.SerialException as e:
            logger.error(f"Read error: {e}")
            self._connected = False
            return None
    
    def available(self) -> int:
        """Check bytes available to read"""
        if not self.is_connected():
            return 0
        
        try:
            with self._lock:
                return self.serial.in_waiting
        except serial.SerialException:
            return 0
    
    def reconnect(self) -> bool:
        """Attempt to reconnect"""
        self.disconnect()
        time.sleep(self.config.reconnect_delay)
        return self.connect()
    
    @staticmethod
    def list_ports() -> List[str]:
        """List available serial ports"""
        return [port.device for port in serial.tools.list_ports.comports()]