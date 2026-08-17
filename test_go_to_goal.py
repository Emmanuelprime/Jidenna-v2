#!/usr/bin/env python3
"""
Simple Go-to-Goal Controller - Direct Heading Control

This controller uses a simpler approach:
1. Turn in place to face the goal
2. Drive straight toward the goal
3. Slow down as you approach

This avoids the curving behavior of Pure Pursuit for point-to-point navigation.

Usage:
    python go_to_goal.py --goal 1.0 1.0 --simulate
    python go_to_goal.py --goal 2.0 3.0 --port /dev/ttyUSB0
"""

import math
import time
import argparse
import logging
import sys
import os
from typing import Tuple, Optional
from dataclasses import dataclass
from enum import Enum

# Add parent directory to path if needed
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from jidenna import RobotAPI, RobotState
except ImportError:
    print("Error: Could not import jidenna module")
    print("Make sure you're running from the correct directory")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class GoToGoalState(Enum):
    """States for go-to-goal controller"""
    IDLE = "IDLE"
    TURNING = "TURNING"          # Turning to face goal
    DRIVING = "DRIVING"          # Driving straight to goal
    FINAL_APPROACH = "FINAL"     # Slow approach
    GOAL_REACHED = "GOAL_REACHED"
    ERROR = "ERROR"


@dataclass
class GoToGoalConfig:
    """Configuration for go-to-goal controller"""
    # Speed settings
    max_linear_speed: float = 0.25      # m/s
    max_angular_speed: float = 0.8      # rad/s
    approach_speed: float = 0.08        # m/s (final approach)
    min_speed: float = 0.02             # m/s (minimum to overcome friction)
    
    # Turning settings
    turn_kp: float = 2.5                # Proportional gain for turning
    turn_tolerance: float = 0.05        # radians (2.86°)
    turn_timeout: float = 5.0           # seconds
    
    # Goal settings
    position_tolerance: float = 0.12    # meters
    slowdown_distance: float = 0.4      # meters (start slowing down)
    approach_distance: float = 0.25     # meters (final approach)
    
    # Control loop
    control_rate: float = 20.0          # Hz
    
    # Safety
    max_heading_error: float = math.radians(170)  # Max angle to turn


class GoToGoal:
    """
    Simple Go-to-Goal controller with direct heading control
    
    Strategy:
    1. Calculate angle to goal
    2. Turn in place to face goal
    3. Drive straight to goal
    4. Slow down near goal
    """
    
    def __init__(self, 
                 robot_api: RobotAPI,
                 config: Optional[GoToGoalConfig] = None):
        """
        Initialize Go-to-Goal controller
        
        Args:
            robot_api: Connected RobotAPI instance
            config: Configuration (optional)
        """
        self.robot = robot_api
        self.config = config if config else GoToGoalConfig()
        
        # State
        self.state = GoToGoalState.IDLE
        self.goal = (0.0, 0.0)
        self.distance_to_goal = float('inf')
        self.heading_error = 0.0
        self.elapsed_time = 0.0
        
        # Simulation support
        self.simulate = False
        self.sim_pose = (0.0, 0.0, 0.0)
        self.sim_last_time = time.time()
        
        # Internal state
        self._start_time = 0.0
        self._turn_start_time = 0.0
        self._last_command = (0.0, 0.0)
        
        # For heading filtering
        self._heading_history = []
        self._history_size = 5
        
        logger.info("GoToGoal initialized (Direct Heading Control)")
        logger.info(f"Max speed: {self.config.max_linear_speed:.2f} m/s")
        logger.info(f"Turn tolerance: {math.degrees(self.config.turn_tolerance):.1f}°")
    
    def set_goal(self, x: float, y: float) -> bool:
        """
        Set goal position
        
        Args:
            x: Goal X position (meters)
            y: Goal Y position (meters)
            
        Returns:
            True if goal set successfully
        """
        self.goal = (x, y)
        self.state = GoToGoalState.TURNING
        self._start_time = time.time()
        self._turn_start_time = time.time()
        
        # Get current pose
        if self.simulate:
            pose = self.sim_pose
        else:
            state = self.robot.get_state()
            pose = (state.x, state.y, state.heading)
        
        # Calculate initial distance and angle
        self.distance_to_goal = self._distance_to_goal(pose[0], pose[1])
        self.heading_error = self._angle_to_goal(pose)
        
        logger.info(f"🎯 Goal set to ({x:.2f}, {y:.2f})")
        logger.info(f"📍 Starting distance: {self.distance_to_goal:.3f}m")
        logger.info(f"🧭 Initial heading error: {math.degrees(self.heading_error):.1f}°")
        
        return True
    
    def update(self, pose: Tuple[float, float, float], 
               imu_heading: Optional[float] = None) -> Tuple[float, float]:
        """
        Update controller and compute velocity command
        
        Args:
            pose: (x, y, heading) robot pose
            imu_heading: IMU heading (optional, for fusion)
            
        Returns:
            (v, w) velocity command
        """
        # Use IMU if available and valid
        heading = pose[2]
        if imu_heading is not None and not self.simulate:
            # Simple fusion: use IMU if discrepancy is reasonable
            diff = self._normalize_angle(imu_heading - heading)
            if abs(diff) < math.radians(30):
                heading = imu_heading  # Trust IMU
            else:
                # Blend if discrepancy is moderate
                blend = 0.5 if abs(diff) < math.radians(60) else 0.0
                if blend > 0:
                    heading = heading + blend * diff
        
        x, y, _ = pose
        pose_with_heading = (x, y, heading)
        
        # Update distance to goal
        self.distance_to_goal = self._distance_to_goal(x, y)
        self.elapsed_time = time.time() - self._start_time
        
        # Check if goal reached
        if self.distance_to_goal < self.config.position_tolerance:
            self.state = GoToGoalState.GOAL_REACHED
            logger.info("✅ Goal reached!")
            return (0.0, 0.0)
        
        # Calculate heading error to goal
        angle_to_goal = math.atan2(self.goal[1] - y, self.goal[0] - x)
        self.heading_error = self._normalize_angle(angle_to_goal - heading)
        
        # State machine
        if self.state == GoToGoalState.TURNING:
            return self._handle_turning(pose_with_heading)
        elif self.state == GoToGoalState.DRIVING:
            return self._handle_driving(pose_with_heading)
        elif self.state == GoToGoalState.FINAL_APPROACH:
            return self._handle_final_approach(pose_with_heading)
        else:
            return (0.0, 0.0)
    
    def _handle_turning(self, pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """
        Handle turning state: turn in place to face goal
        
        Args:
            pose: (x, y, heading) robot pose
            
        Returns:
            (v, w) velocity command
        """
        x, y, heading = pose
        
        # Check if we've been turning too long
        if time.time() - self._turn_start_time > self.config.turn_timeout:
            logger.warning(f"Turn timeout! Heading error: {math.degrees(self.heading_error):.1f}°")
            # Force proceed anyway
            self.state = GoToGoalState.DRIVING
            return self._handle_driving(pose)
        
        # Check if we're facing the goal
        abs_error = abs(self.heading_error)
        
        if abs_error < self.config.turn_tolerance:
            logger.info(f"✅ Facing goal! Error: {math.degrees(abs_error):.1f}°")
            self.state = GoToGoalState.DRIVING
            return self._handle_driving(pose)
        
        # Calculate angular velocity (P-controller with deadzone)
        if abs_error < 0.1:  # Small error, gentle correction
            w = self.config.turn_kp * self.heading_error * 0.5
        else:
            w = self.config.turn_kp * self.heading_error
        
        # Limit angular velocity
        max_w = self.config.max_angular_speed
        w = max(-max_w, min(w, max_w))
        
        # If heading error is large, turn faster
        if abs_error > math.radians(90):
            w = math.copysign(max_w, w)
        
        # Zero linear velocity (turn in place)
        v = 0.0
        
        # Log occasionally
        if abs_error > math.radians(5):
            logger.debug(f"Turning: error={math.degrees(self.heading_error):.1f}°, w={w:.3f}")
        
        self._last_command = (v, w)
        return (v, w)
    
    def _handle_driving(self, pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """
        Handle driving state: drive straight to goal
        
        Args:
            pose: (x, y, heading) robot pose
            
        Returns:
            (v, w) velocity command
        """
        x, y, heading = pose
        
        # Check if we need to turn more (heading error grew)
        if abs(self.heading_error) > math.radians(15):
            logger.info(f"Re-turning: error={math.degrees(self.heading_error):.1f}°")
            self.state = GoToGoalState.TURNING
            self._turn_start_time = time.time()
            return self._handle_turning(pose)
        
        # Check if we're in final approach
        if self.distance_to_goal < self.config.approach_distance:
            self.state = GoToGoalState.FINAL_APPROACH
            return self._handle_final_approach(pose)
        
        # Calculate linear velocity with slowdown
        if self.distance_to_goal < self.config.slowdown_distance:
            # Slow down as we approach
            factor = self.distance_to_goal / self.config.slowdown_distance
            v = self.config.max_linear_speed * max(0.2, factor)
        else:
            v = self.config.max_linear_speed
        
        # Add small correction for heading error while driving
        w = 0.8 * self.heading_error
        
        # Limit angular velocity while driving (gentle corrections)
        max_w = self.config.max_angular_speed * 0.5
        w = max(-max_w, min(w, max_w))
        
        # Ensure minimum speed to move
        v = max(self.config.min_speed, v)
        
        # Log progress
        if int(self.distance_to_goal * 10) % 5 == 0:
            logger.debug(f"Driving: dist={self.distance_to_goal:.3f}m, "
                        f"heading_err={math.degrees(self.heading_error):.1f}°, "
                        f"v={v:.3f}, w={w:.3f}")
        
        self._last_command = (v, w)
        return (v, w)
    
    def _handle_final_approach(self, pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """
        Handle final approach: very slow, precise movement to goal
        
        Args:
            pose: (x, y, heading) robot pose
            
        Returns:
            (v, w) velocity command
        """
        x, y, heading = pose
        
        # Very slow approach speed
        speed_factor = max(0.3, self.distance_to_goal / self.config.approach_distance)
        v = self.config.approach_speed * speed_factor
        v = max(self.config.min_speed, min(v, self.config.approach_speed))
        
        # Fine heading correction
        w = 1.5 * self.heading_error
        
        # Very limited angular velocity in final approach
        max_w = 0.3
        w = max(-max_w, min(w, max_w))
        
        # If very close, just go straight
        if self.distance_to_goal < 0.08:
            w = 0.0
        
        logger.debug(f"Final approach: dist={self.distance_to_goal:.3f}m, "
                    f"heading_err={math.degrees(self.heading_error):.1f}°, "
                    f"v={v:.3f}, w={w:.3f}")
        
        self._last_command = (v, w)
        return (v, w)
    
    def run(self, goal_x: float, goal_y: float, 
            max_time: float = 60.0) -> bool:
        """
        Run navigation to goal
        
        Args:
            goal_x: Goal X position
            goal_y: Goal Y position
            max_time: Maximum time in seconds
            
        Returns:
            True if goal reached, False otherwise
        """
        self.set_goal(goal_x, goal_y)
        
        start_time = time.time()
        last_log = time.time()
        
        logger.info("🚀 Starting navigation...")
        logger.info("Press Ctrl+C to stop")
        
        try:
            while time.time() - start_time < max_time:
                # Get robot pose
                if self.simulate:
                    pose = self.sim_pose
                    imu_heading = None
                else:
                    state = self.robot.get_state()
                    if not state.is_valid():
                        logger.error("Invalid robot state!")
                        break
                    pose = (state.x, state.y, state.heading)
                    imu_heading = state.imu_heading if state.is_imu_valid() else None
                
                # Update controller
                v, w = self.update(pose, imu_heading)
                
                # Send command
                if self.simulate:
                    self._update_simulation(v, w)
                else:
                    self.robot.set_velocity(v, w)
                
                # Log status
                if time.time() - last_log > 1.0:
                    logger.info(f"📍 dist={self.distance_to_goal:.3f}m, "
                               f"heading_err={math.degrees(self.heading_error):.1f}°, "
                               f"state={self.state.value}, "
                               f"v={v:.3f}, w={w:.3f}")
                    last_log = time.time()
                
                # Check if done
                if self.state == GoToGoalState.GOAL_REACHED:
                    logger.info("🎯 Goal reached successfully!")
                    self.robot.stop()
                    return True
                
                time.sleep(1.0 / self.config.control_rate)
                
            logger.warning(f"⏱️ Timeout after {max_time}s")
            self.robot.stop()
            return False
            
        except KeyboardInterrupt:
            logger.info("⏹️ Navigation interrupted")
            self.robot.stop()
            return False
        except Exception as e:
            logger.error(f"Error: {e}")
            self.robot.stop()
            return False
    
    def _distance_to_goal(self, x: float, y: float) -> float:
        """Calculate distance to goal"""
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        return math.sqrt(dx*dx + dy*dy)
    
    def _angle_to_goal(self, pose: Tuple[float, float, float]) -> float:
        """Calculate heading error to goal"""
        x, y, heading = pose
        angle_to_goal = math.atan2(self.goal[1] - y, self.goal[0] - x)
        return self._normalize_angle(angle_to_goal - heading)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _update_simulation(self, v: float, w: float):
        """Update simulated robot pose"""
        current_time = time.time()
        dt = min(current_time - self.sim_last_time, 0.1)
        self.sim_last_time = current_time
        
        x, y, heading = self.sim_pose
        
        # Update heading
        heading += w * dt
        heading = self._normalize_angle(heading)
        
        # Update position
        x += v * math.cos(heading) * dt
        y += v * math.sin(heading) * dt
        
        self.sim_pose = (x, y, heading)
    
    def stop(self):
        """Stop the robot"""
        self.robot.stop()
        self.state = GoToGoalState.IDLE
    
    def get_status(self) -> dict:
        """Get current status"""
        return {
            'state': self.state.value,
            'goal': self.goal,
            'distance': self.distance_to_goal,
            'heading_error': self.heading_error,
            'heading_error_deg': math.degrees(self.heading_error),
            'elapsed_time': self.elapsed_time,
            'last_command': self._last_command
        }


class SimulatedRobot:
    """Simple robot simulation"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = time.time()
    
    def set_velocity(self, v: float, w: float):
        self.v = v
        self.w = w
    
    def get_state(self):
        current_time = time.time()
        dt = min(current_time - self.last_time, 0.1)
        self.last_time = current_time
        
        # Update simulation
        self.heading += self.w * dt
        self.heading = self._normalize_angle(self.heading)
        self.x += self.v * math.cos(self.heading) * dt
        self.y += self.v * math.sin(self.heading) * dt
        
        class State:
            def __init__(self, x, y, heading):
                self.x = x
                self.y = y
                self.heading = heading
                self.v = 0.0
                self.w = 0.0
            
            def is_valid(self):
                return True
            
            def is_imu_valid(self):
                return False
            
            @property
            def imu_heading(self):
                return self.heading
        
        return State(self.x, self.y, self.heading)
    
    def stop(self):
        self.v = 0.0
        self.w = 0.0
    
    def disconnect(self):
        pass
    
    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="Go-to-Goal with Direct Heading Control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Simulation
  python go_to_goal.py --goal 1.0 1.0 --simulate
  
  # Real robot
  python go_to_goal.py --goal 2.0 3.0 --port /dev/ttyUSB0
  
  # Custom speed
  python go_to_goal.py --goal 1.5 0.5 --port COM3 --speed 0.3
        """
    )
    
    parser.add_argument(
        '--goal', type=float, nargs=2, required=True,
        help='Goal position (x y) in meters'
    )
    parser.add_argument(
        '--port', type=str,
        help='Serial port for robot'
    )
    parser.add_argument(
        '--simulate', action='store_true',
        help='Run in simulation mode'
    )
    parser.add_argument(
        '--speed', type=float, default=0.25,
        help='Maximum linear velocity (m/s)'
    )
    parser.add_argument(
        '--turn-speed', type=float, default=0.8,
        help='Maximum angular velocity (rad/s)'
    )
    parser.add_argument(
        '--timeout', type=float, default=60.0,
        help='Navigation timeout (seconds)'
    )
    parser.add_argument(
        '--verbose', action='store_true',
        help='Enable verbose logging'
    )
    parser.add_argument(
        '--list-ports', action='store_true',
        help='List available serial ports'
    )
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # List ports
    if args.list_ports:
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            print("\nAvailable Serial Ports:")
            for port in ports:
                print(f"  {port.device}: {port.description}")
        except ImportError:
            print("pyserial not installed")
        return
    
    # Setup robot
    if args.simulate:
        logger.info("🔬 SIMULATION MODE")
        robot = SimulatedRobot()
    else:
        if not args.port:
            logger.error("--port required for real robot mode")
            return 1
        
        robot = RobotAPI(args.port)
        if not robot.connect():
            logger.error(f"Failed to connect to robot on {args.port}")
            return 1
        logger.info(f"Connected to robot on {args.port}")
    
    # Create config
    config = GoToGoalConfig()
    config.max_linear_speed = args.speed
    config.max_angular_speed = args.turn_speed
    
    # Create controller
    gtg = GoToGoal(robot, config)
    
    # Set simulation flag
    gtg.simulate = args.simulate
    if args.simulate:
        gtg.sim_pose = (0.0, 0.0, 0.0)
    
    try:
        # Get current position
        state = robot.get_state()
        logger.info(f"📍 Starting at: ({state.x:.2f}, {state.y:.2f})")
        
        # Run navigation
        goal_x, goal_y = args.goal
        success = gtg.run(goal_x, goal_y, max_time=args.timeout)
        
        # Print results
        print("\n" + "="*50)
        print("NAVIGATION RESULTS")
        print("="*50)
        status = gtg.get_status()
        print(f"Goal: ({status['goal'][0]:.2f}, {status['goal'][1]:.2f})")
        print(f"Final Distance: {status['distance']:.3f}m")
        print(f"Heading Error: {status['heading_error_deg']:.1f}°")
        print(f"Time: {status['elapsed_time']:.1f}s")
        print(f"Success: {'✅ YES' if success else '❌ NO'}")
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        logger.info("\nInterrupted by user")
        return 1
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if not args.simulate:
            robot.stop()
            robot.disconnect()
            logger.info("Robot disconnected")


if __name__ == "__main__":
    sys.exit(main())