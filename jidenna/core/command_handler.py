"""
Command handling with response tracking
"""

import time
import threading
from typing import Optional, Dict, Any
from dataclasses import dataclass
import queue
import logging

logger = logging.getLogger(__name__)


@dataclass
class CommandResponse:
    """Command response structure"""
    success: bool
    response: Optional[str] = None
    error: Optional[str] = None
    timestamp: float = time.time()


class CommandHandler:
    """Handles commands to robot with response tracking"""
    
    def __init__(self, serial_interface):
        self.serial = serial_interface
        self._lock = threading.Lock()
        self._pending_commands: Dict[str, queue.Queue] = {}
        self._sequence = 0
        
    def send_command(self, command: str) -> CommandResponse:
        """Send command and wait for response"""
        if not self.serial.is_connected():
            return CommandResponse(success=False, error="Not connected")
        
        try:
            # Send command
            if not self.serial.write(command + "\n"):
                return CommandResponse(success=False, error="Write failed")
            
            # Wait for ACK
            start_time = time.time()
            while time.time() - start_time < 2.0:
                if self.serial.available():
                    line = self.serial.readline()
                    if line and line.startswith("ACK"):
                        parts = line.split(',')
                        if len(parts) >= 3:
                            cmd_type = parts[1]
                            if cmd_type in command:
                                return CommandResponse(success=True, response=line)
                
                time.sleep(0.01)
            
            return CommandResponse(success=False, error="No ACK received")
            
        except Exception as e:
            logger.error(f"Command error: {e}")
            return CommandResponse(success=False, error=str(e))