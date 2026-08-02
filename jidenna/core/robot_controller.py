"""
Advanced ESP32 Robot Controller API - Fixed Version
"""

import time
import threading
import queue
import math
from typing import Dict, List, Optional, Callable, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Try imports
try:
    from .serial_interface import SerialInterface, SerialConfig
    from .telemetry import TelemetryManager, TelemetryData
    from .command_handler import CommandHandler, CommandResponse
except ImportError:
    from serial_interface import SerialInterface, SerialConfig
    from telemetry import TelemetryManager, TelemetryData
    from command_handler import CommandHandler, CommandResponse


class RobotState(Enum):
    """Robot operational states"""
    IDLE = 0
    ACCELERATING = 1
    CRUISING = 2
    DECELERATING = 3
    EMERGENCY_STOP = 4
    CALIBRATING = 5
    ERROR = 6
    PAUSED = 7
    AUTONOMOUS = 8


@dataclass
class RobotStatus:
    """Complete robot status"""
    state: RobotState
    position: Tuple[float, float] = (0.0, 0.0)
    yaw: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    left_speed: float = 0.0
    right_speed: float = 0.0
    left_pwm: int = 0
    right_pwm: int = 0
    battery_voltage: float = 0.0
    errors: List[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)


class RobotController:
    """Main robot controller with plugin architecture"""
    
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        self.serial_config = SerialConfig(port=port, baud=baud, timeout=timeout)
        self.serial = SerialInterface(self.serial_config)
        self.telemetry_manager = TelemetryManager()
        self.command_handler = CommandHandler(self.serial)
        
        # State
        self._state = RobotState.IDLE
        self._status = RobotStatus(state=RobotState.IDLE)
        self._connected = False
        self._running = False
        self._lock = threading.RLock()
        
        # Plugin system
        self._plugins: Dict[str, object] = {}
        self._plugin_threads: List[threading.Thread] = []
        
        # Event system
        self._event_handlers: Dict[str, List[Callable]] = {
            'on_telemetry': [],
            'on_state_change': [],
            'on_error': [],
            'on_command': [],
            'on_status': []
        }
        
        # Command queue
        self._command_queue: queue.Queue = queue.Queue()
        self._command_thread: Optional[threading.Thread] = None
        
        # Telemetry thread
        self._telemetry_thread: Optional[threading.Thread] = None
        
        # Statistics
        self._telemetry_count = 0
        
        logger.info(f"RobotController initialized for port {port}")
    
    # ─── CONNECTION MANAGEMENT ──────────────────────────────────────────────
    
    def connect(self, timeout: float = 5.0) -> bool:
        """Connect to robot"""
        try:
            self.serial.connect()
            self._connected = True
            self._running = True
            
            # Start telemetry thread
            self._start_telemetry()
            
            # Wait for first telemetry packet
            if self.telemetry_manager.wait_for_data(timeout):
                logger.info("Robot connected successfully - telemetry received")
                self._update_state(RobotState.IDLE)
                return True
            else:
                logger.warning("Connected but no telemetry received - check robot output")
                # Still return True as we're connected, just no data yet
                return True
                
        except Exception as e:
            logger.error(f"Failed to connect to robot: {e}")
            self._connected = False
            return False
    
    def disconnect(self):
        """Disconnect from robot"""
        self._running = False
        self._connected = False
        
        # Stop all threads
        self._stop_telemetry()
        self._stop_command_thread()
        self._stop_plugin_threads()
        
        # Send stop command
        try:
            self.emergency_stop()
        except:
            pass
        
        self.serial.disconnect()
        logger.info("Robot disconnected")
    
    def is_connected(self) -> bool:
        return self._connected and self.serial.is_connected()
    
    # ─── TELEMETRY ────────────────────────────────────────────────────────────
    
    def _start_telemetry(self):
        """Start telemetry reading thread"""
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            return
            
        self._running = True
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_thread.start()
        logger.debug("Telemetry thread started")
    
    def _stop_telemetry(self):
        """Stop telemetry thread"""
        self._running = False
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            self._telemetry_thread.join(timeout=2.0)
            logger.debug("Telemetry thread stopped")
    
    def _telemetry_loop(self):
        """Main telemetry reading loop"""
        consecutive_errors = 0
        
        while self._running and self._connected:
            try:
                if self.serial.is_connected() and self.serial.available():
                    line = self.serial.readline()
                    if line:
                        self._telemetry_count += 1
                        
                        # Log every 50th telemetry packet for debugging
                        if self._telemetry_count % 50 == 0:
                            logger.debug(f"Received telemetry #{self._telemetry_count}: {line[:50]}...")
                        
                        if line.startswith("CNT"):
                            data = self.telemetry_manager.parse_telemetry(line)
                            if data:
                                consecutive_errors = 0
                                # Update status
                                with self._lock:
                                    self._status.position = (data.x, data.y)
                                    self._status.yaw = data.yaw
                                    self._status.linear_velocity = data.linear_velocity
                                    self._status.angular_velocity = data.angular_velocity
                                    self._status.left_speed = data.left_speed
                                    self._status.right_speed = data.right_speed
                                    self._status.left_pwm = data.left_pwm
                                    self._status.right_pwm = data.right_pwm
                                    self._status.timestamp = data.timestamp
                                
                                # Trigger events
                                self._trigger_event('on_telemetry', data)
                                
                        elif line.startswith("ACK"):
                            logger.debug(f"ACK: {line}")
                        elif line.startswith("WARN"):
                            logger.warning(f"Robot warning: {line}")
                        elif line.startswith("READY"):
                            logger.info("Robot ready")
                            self._update_state(RobotState.IDLE)
                        elif line.startswith("STATUS"):
                            self._parse_status(line)
                        elif line.startswith("ERROR"):
                            logger.error(f"Robot error: {line}")
                            self._trigger_event('on_error', line)
                        elif line.strip():
                            # Log any other output for debugging
                            logger.debug(f"Robot output: {line}")
                            
                else:
                    # Check connection
                    if self._connected and not self.serial.is_connected():
                        self._connected = False
                        self._trigger_event('on_error', "Connection lost")
                        logger.error("Connection lost to robot")
                        break
                    
                    # Small delay to prevent CPU spinning
                    time.sleep(0.01)
                    
            except Exception as e:
                consecutive_errors += 1
                logger.error(f"Error in telemetry loop: {e}")
                if consecutive_errors > 10:
                    logger.error("Too many errors in telemetry loop - exiting")
                    break
                time.sleep(0.1)
    
    def get_telemetry(self) -> Optional[TelemetryData]:
        """Get latest telemetry data"""
        return self.telemetry_manager.get_latest()
    
    def get_status(self) -> RobotStatus:
        """Get robot status"""
        with self._lock:
            return self._status
    
    def get_telemetry_count(self) -> int:
        """Get number of telemetry packets received"""
        return self._telemetry_count
    
    # ─── COMMANDS ────────────────────────────────────────────────────────────
    
    def send_velocity(self, linear: float, angular: float) -> CommandResponse:
        """Send velocity command to robot"""
        if not self.is_connected():
            return CommandResponse(success=False, error="Not connected")
        
        # Clamp values
        linear = max(-1.2, min(1.2, linear))
        angular = max(-2.0, min(2.0, angular))
        
        cmd = f"V{linear:.3f},{angular:.3f}"
        response = self.command_handler.send_command(cmd)
        
        if response.success:
            self._trigger_event('on_command', {'linear': linear, 'angular': angular})
        
        return response
    
    def move_forward(self, speed: float = 0.3, duration: Optional[float] = None) -> bool:
        """Move robot forward"""
        if duration:
            self.send_velocity(speed, 0.0)
            time.sleep(duration)
            self.send_velocity(0.0, 0.0)
            return True
        else:
            return self.send_velocity(speed, 0.0).success
    
    def move_backward(self, speed: float = 0.3, duration: Optional[float] = None) -> bool:
        """Move robot backward"""
        return self.move_forward(-speed, duration)
    
    def turn_left(self, speed: float = 0.5, duration: Optional[float] = None) -> bool:
        """Turn robot left"""
        if duration:
            self.send_velocity(0.0, speed)
            time.sleep(duration)
            self.send_velocity(0.0, 0.0)
            return True
        else:
            return self.send_velocity(0.0, speed).success
    
    def turn_right(self, speed: float = 0.5, duration: Optional[float] = None) -> bool:
        """Turn robot right"""
        return self.turn_left(-speed, duration)
    
    def turn_degrees(self, degrees: float, speed: float = 0.5) -> bool:
        """Turn robot by specified degrees"""
        if not self.is_connected():
            return False
        
        duration = abs(degrees) / (speed * 57.2958)
        
        # Wait for robot to be stable
        time.sleep(0.1)
        
        # Turn
        angular = speed if degrees > 0 else -speed
        self.send_velocity(0.0, angular)
        time.sleep(duration)
        self.send_velocity(0.0, 0.0)
        
        return True
    
    def move_distance(self, distance: float, speed: float = 0.3) -> bool:
        """Move robot by specified distance"""
        if not self.is_connected():
            return False
        
        duration = abs(distance) / abs(speed)
        
        time.sleep(0.1)
        
        self.send_velocity(speed if distance > 0 else -speed, 0.0)
        time.sleep(duration)
        self.send_velocity(0.0, 0.0)
        
        return True
    
    def stop(self):
        """Stop robot motion"""
        self.send_velocity(0.0, 0.0)
    
    def emergency_stop(self):
        """Emergency stop"""
        self.send_velocity(0.0, 0.0)
        with self._lock:
            self._state = RobotState.EMERGENCY_STOP
        self._trigger_event('on_state_change', self._state)
        logger.warning("Emergency stop triggered")
    
    def calibrate(self):
        """Calibrate IMU"""
        if not self.is_connected():
            return False
        self.send_command("CALIBRATE")
        return True
    
    def reset(self):
        """Reset robot"""
        if not self.is_connected():
            return False
        self.send_command("RESET")
        return True
    
    def send_command(self, command: str) -> CommandResponse:
        """Send raw command to robot"""
        if not self.is_connected():
            return CommandResponse(success=False, error="Not connected")
        return self.command_handler.send_command(command)
    
    def _stop_command_thread(self):
        """Stop command execution thread"""
        if self._command_thread and self._command_thread.is_alive():
            self._command_thread.join(timeout=1.0)
            logger.debug("Command thread stopped")
    
    # ─── AUTONOMOUS COMMANDS ────────────────────────────────────────────────
    
    def go_to_position(self, target_x: float, target_y: float, speed: float = 0.3, 
                       tolerance: float = 0.05) -> bool:
        """Go to target position"""
        max_attempts = 100
        
        for _ in range(max_attempts):
            telemetry = self.get_telemetry()
            if not telemetry:
                time.sleep(0.1)
                continue
            
            dx = target_x - telemetry.x
            dy = target_y - telemetry.y
            distance = (dx**2 + dy**2)**0.5
            
            if distance < tolerance:
                self.stop()
                return True
            
            # Calculate angle to target
            target_angle = math.atan2(dy, dx)
            current_angle = telemetry.yaw * math.pi / 180.0
            
            angle_diff = target_angle - current_angle
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            linear = min(speed, distance * 2)
            angular = max(-0.5, min(0.5, angle_diff * 2))
            
            self.send_velocity(linear if abs(angle_diff) < 0.1 else linear * 0.5, angular)
            time.sleep(0.1)
        
        self.stop()
        return False
    
    # ─── PLUGIN SYSTEM ──────────────────────────────────────────────────────
    
    def register_plugin(self, plugin) -> bool:
        """Register a plugin"""
        plugin_name = plugin.get_name()
        
        if plugin_name in self._plugins:
            logger.warning(f"Plugin {plugin_name} already registered")
            return False
        
        self._plugins[plugin_name] = plugin
        plugin.set_controller(self)
        
        try:
            plugin.initialize()
            logger.info(f"Plugin {plugin_name} registered successfully")
            
            if plugin.requires_thread():
                thread = threading.Thread(target=plugin.run, daemon=True)
                thread.start()
                self._plugin_threads.append(thread)
            
            return True
        except Exception as e:
            logger.error(f"Failed to initialize plugin {plugin_name}: {e}")
            del self._plugins[plugin_name]
            return False
    
    def unregister_plugin(self, plugin_name: str) -> bool:
        """Unregister a plugin"""
        if plugin_name not in self._plugins:
            return False
        
        try:
            plugin = self._plugins[plugin_name]
            plugin.shutdown()
            del self._plugins[plugin_name]
            logger.info(f"Plugin {plugin_name} unregistered")
            return True
        except Exception as e:
            logger.error(f"Failed to unregister plugin {plugin_name}: {e}")
            return False
    
    def get_plugin(self, plugin_name: str) -> Optional[object]:
        """Get a registered plugin"""
        return self._plugins.get(plugin_name)
    
    def _stop_plugin_threads(self):
        """Stop all plugin threads"""
        for thread in self._plugin_threads:
            if thread.is_alive():
                thread.join(timeout=2.0)
        self._plugin_threads.clear()
    
    # ─── EVENT SYSTEM ──────────────────────────────────────────────────────
    
    def on_telemetry(self, callback: Callable):
        """Decorator for telemetry event"""
        self._event_handlers['on_telemetry'].append(callback)
        return callback
    
    def on_state_change(self, callback: Callable):
        """Decorator for state change event"""
        self._event_handlers['on_state_change'].append(callback)
        return callback
    
    def on_error(self, callback: Callable):
        """Decorator for error event"""
        self._event_handlers['on_error'].append(callback)
        return callback
    
    def on_command(self, callback: Callable):
        """Decorator for command event"""
        self._event_handlers['on_command'].append(callback)
        return callback
    
    def _trigger_event(self, event_name: str, *args, **kwargs):
        """Trigger an event"""
        handlers = self._event_handlers.get(event_name, [])
        for handler in handlers:
            try:
                handler(*args, **kwargs)
            except Exception as e:
                logger.error(f"Error in event handler {event_name}: {e}")
    
    # ─── STATE MANAGEMENT ──────────────────────────────────────────────────
    
    def _update_state(self, state: RobotState):
        """Update robot state"""
        with self._lock:
            if self._state != state:
                self._state = state
                self._status.state = state
                self._trigger_event('on_state_change', state)
    
    def get_state(self) -> RobotState:
        """Get robot state"""
        with self._lock:
            return self._state
    
    def _parse_status(self, line: str):
        """Parse status message"""
        try:
            parts = line.split(',')
            for part in parts:
                if '=' in part:
                    key, value = part.split('=')
                    if key == 'state':
                        self._update_state(RobotState(int(value)))
        except Exception as e:
            logger.error(f"Error parsing status: {e}")
    
    # ─── CONTEXT MANAGER ──────────────────────────────────────────────────
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False