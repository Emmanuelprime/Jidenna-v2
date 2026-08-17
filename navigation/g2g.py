#!/usr/bin/env python3
"""
GoToGoal Controller - Direct Heading Control with PID

This module provides a go-to-goal controller that uses direct heading control
with PID to navigate to a goal point. It integrates with your existing
RobotAPI and RobotState but does NOT use Pure Pursuit.

The robot will:
1. Turn in place to face the goal (PID controlled)
2. Drive straight to the goal
3. Slow down as it approaches
4. Stop precisely at the goal

Usage:
    from go_to_goal import GoToGoal, GoToGoalConfig
    from jidenna import RobotAPI
    
    robot = RobotAPI('/dev/ttyUSB0')
    robot.connect()
    
    gtg = GoToGoal(robot)
    gtg.go_to(2.0, 2.0)
"""

import math
import time
import logging
from enum import Enum
from dataclasses import dataclass, field
from typing import Tuple, Optional, Callable

logger = logging.getLogger(__name__)


class GoToGoalState(Enum):
    """States for go-to-goal controller"""
    IDLE = "IDLE"
    TURNING = "TURNING"           # Turning to face goal
    DRIVING = "DRIVING"           # Driving straight to goal
    FINAL_APPROACH = "FINAL"      # Slow approach
    GOAL_REACHED = "GOAL_REACHED"
    ERROR = "ERROR"
    STOPPED = "STOPPED"


@dataclass
class PIDConfig:
    """PID controller configuration"""
    kp: float = 2.5               # Proportional gain
    ki: float = 0.1               # Integral gain
    kd: float = 0.05              # Derivative gain
    integral_clamp: float = 0.5   # Clamp integral term
    output_limit: float = 1.0     # Max output


@dataclass
class GoToGoalConfig:
    """Configuration for GoToGoal controller"""
    # Speed settings
    max_linear_speed: float = 0.25      # m/s
    max_angular_speed: float = 0.8      # rad/s
    min_linear_speed: float = 0.02      # m/s (minimum to overcome friction)
    
    # PID settings for turning
    turn_pid: PIDConfig = field(default_factory=lambda: PIDConfig(
        kp=2.5, ki=0.1, kd=0.05, integral_clamp=0.5, output_limit=0.8
    ))
    
    # PID settings for driving correction
    drive_pid: PIDConfig = field(default_factory=lambda: PIDConfig(
        kp=0.8, ki=0.02, kd=0.02, integral_clamp=0.1, output_limit=0.3
    ))
    
    # Goal settings
    position_tolerance: float = 0.10    # meters
    turn_tolerance: float = 0.05        # radians (2.86 deg)
    slowdown_distance: float = 0.5      # meters (start slowing down)
    approach_distance: float = 0.25     # meters (final approach)
    
    # Timing
    turn_timeout: float = 8.0           # seconds
    control_rate: float = 20.0          # Hz
    max_time: float = 60.0              # seconds
    
    # IMU fusion
    use_imu: bool = True
    imu_weight: float = 0.9             # 90% IMU, 10% odometry
    
    # Safety
    max_heading_error: float = math.radians(170)  # Max angle to turn
    stuck_threshold: float = 3.0        # seconds without progress


class PIDController:
    """Simple PID controller"""
    
    def __init__(self, config: PIDConfig):
        self.config = config
        self.reset()
    
    def reset(self):
        """Reset PID state"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
    
    def update(self, error: float, dt: float) -> float:
        """
        Update PID controller
        
        Args:
            error: Current error (setpoint - measurement)
            dt: Time step in seconds
            
        Returns:
            Control output
        """
        if dt <= 0:
            return 0.0
        
        # Proportional
        p = self.config.kp * error
        
        # Integral (with anti-windup)
        self._integral += error * dt
        self._integral = max(-self.config.integral_clamp, 
                            min(self._integral, self.config.integral_clamp))
        i = self.config.ki * self._integral
        
        # Derivative
        if self._prev_time is not None and dt > 0:
            derivative = (error - self._prev_error) / dt
        else:
            derivative = 0.0
        d = self.config.kd * derivative
        
        # Output
        output = p + i + d
        
        # Clamp output
        output = max(-self.config.output_limit, 
                    min(output, self.config.output_limit))
        
        # Store for next iteration
        self._prev_error = error
        self._prev_time = time.time()
        
        return output


class GoToGoal:
    """
    Go-to-Goal controller with direct heading control
    
    Uses PID controllers for precise heading control and path following.
    Does NOT use Pure Pursuit - this is a simpler, more direct approach.
    """
    
    def __init__(self, 
                 robot_api,
                 config: Optional[GoToGoalConfig] = None):
        """
        Initialize GoToGoal controller
        
        Args:
            robot_api: RobotAPI instance (from jidenna)
            config: Configuration (optional)
        """
        self.robot = robot_api
        self.config = config if config else GoToGoalConfig()
        
        # PID controllers
        self.turn_pid = PIDController(self.config.turn_pid)
        self.drive_pid = PIDController(self.config.drive_pid)
        
        # State
        self.state = GoToGoalState.IDLE
        self.goal = (0.0, 0.0)
        self.distance_to_goal = float('inf')
        self.heading_error = 0.0
        self.elapsed_time = 0.0
        self.total_distance_traveled = 0.0
        
        # Internal state
        self._start_time = 0.0
        self._turn_start_time = 0.0
        self._last_pose = (0.0, 0.0, 0.0)
        self._last_command = (0.0, 0.0)
        self._prev_distance = float('inf')
        self._stuck_start_time = 0.0
        
        # Logging
        self._last_log_time = 0.0
        
        # Callbacks
        self._status_callback: Optional[Callable] = None
        
        logger.info("GoToGoal Controller initialized")
        logger.info(f"  Max speed: {self.config.max_linear_speed:.2f} m/s")
        logger.info(f"  Turn tolerance: {math.degrees(self.config.turn_tolerance):.1f} deg")
        logger.info(f"  Position tolerance: {self.config.position_tolerance:.3f}m")
        logger.info(f"  IMU fusion: {'ENABLED' if self.config.use_imu else 'DISABLED'}")
    
    def go_to(self, 
              goal_x: float, 
              goal_y: float,
              max_time: Optional[float] = None,
              callback: Optional[Callable] = None) -> bool:
        """
        Navigate to goal position
        
        Args:
            goal_x: Goal X position (meters)
            goal_y: Goal Y position (meters)
            max_time: Maximum time (overrides config)
            callback: Optional callback called each loop with status dict
            
        Returns:
            True if goal reached, False otherwise
        """
        self._status_callback = callback
        
        # Set goal
        if not self._set_goal(goal_x, goal_y):
            return False
        
        # If already at goal
        if self.state == GoToGoalState.GOAL_REACHED:
            return True
        
        # Use provided max_time or config
        max_time = max_time if max_time is not None else self.config.max_time
        
        start_time = time.time()
        self._start_time = start_time
        self._last_log_time = start_time
        
        logger.info(f"Starting navigation to ({goal_x:.2f}, {goal_y:.2f})")
        logger.info(f"   Max time: {max_time:.1f}s")
        logger.info("   Press Ctrl+C to stop")
        
        try:
            while time.time() - start_time < max_time:
                # Get robot state
                state = self.robot.get_state()
                if not state.is_valid():
                    logger.error("Invalid robot state!")
                    self.state = GoToGoalState.ERROR
                    self.robot.stop()
                    return False
                
                # Get pose and IMU heading
                pose = (state.x, state.y, state.heading)
                imu_heading = state.imu_heading if (self.config.use_imu and state.is_imu_valid()) else None
                
                # Update controller
                v, w = self._update(pose, imu_heading)
                
                # Send velocity command
                self.robot.set_velocity(v, w)
                self._last_command = (v, w)
                
                # Update status
                self.distance_to_goal = self._distance_to_goal(state.x, state.y)
                self.elapsed_time = time.time() - start_time
                
                # Log status
                if time.time() - self._last_log_time > 1.0:
                    self._log_status(state, v, w)
                    self._last_log_time = time.time()
                
                # Callback
                if callback:
                    try:
                        callback(self._get_status_dict(state, v, w))
                    except Exception as e:
                        logger.error(f"Callback error: {e}")
                
                # Check if goal reached
                if self.state == GoToGoalState.GOAL_REACHED:
                    logger.info(f"Goal reached! Distance: {self.distance_to_goal:.3f}m")
                    self.robot.stop()
                    return True
                
                # Check if stuck
                if self._check_stuck(state):
                    logger.warning("Stuck detected! Re-turning...")
                    self.state = GoToGoalState.TURNING
                    self._turn_start_time = time.time()
                    self._stuck_start_time = time.time()
                    self.drive_pid.reset()
                    self.turn_pid.reset()
                
                time.sleep(1.0 / self.config.control_rate)
            
            logger.warning(f"Timeout after {max_time:.1f}s")
            self.robot.stop()
            return False
            
        except KeyboardInterrupt:
            logger.info("Navigation interrupted")
            self.robot.stop()
            return False
        except Exception as e:
            logger.error(f"Navigation error: {e}")
            import traceback
            traceback.print_exc()
            self.robot.stop()
            return False
    
    def _set_goal(self, x: float, y: float) -> bool:
        """Set goal and initialize state"""
        self.goal = (x, y)
        self.state = GoToGoalState.TURNING
        self._start_time = time.time()
        self._turn_start_time = time.time()
        self._stuck_start_time = time.time()
        self.total_distance_traveled = 0.0
        
        # Reset PIDs
        self.turn_pid.reset()
        self.drive_pid.reset()
        
        # Get current pose
        state = self.robot.get_state()
        pose = (state.x, state.y, state.heading)
        self._last_pose = pose
        self._prev_distance = float('inf')
        
        # Calculate initial distance and heading error
        self.distance_to_goal = self._distance_to_goal(pose[0], pose[1])
        self.heading_error = self._angle_to_goal(pose)
        
        # Check if already at goal
        if self.distance_to_goal < self.config.position_tolerance:
            self.state = GoToGoalState.GOAL_REACHED
            logger.info("Already at goal!")
            return True
        
        logger.info(f"Goal set to ({x:.2f}, {y:.2f})")
        logger.info(f"Starting at: ({pose[0]:.3f}, {pose[1]:.3f})")
        logger.info(f"Distance: {self.distance_to_goal:.3f}m")
        logger.info(f"Heading error: {math.degrees(self.heading_error):.1f} deg")
        
        return True
    
    def _update(self, pose: Tuple[float, float, float], 
                imu_heading: Optional[float] = None) -> Tuple[float, float]:
        """
        Update controller
        
        Args:
            pose: (x, y, heading) from odometry
            imu_heading: IMU heading (optional)
            
        Returns:
            (v, w) velocity command
        """
        # Fuse heading with IMU
        heading = self._fuse_heading(pose[2], imu_heading)
        fused_pose = (pose[0], pose[1], heading)
        
        # Update distance traveled
        dx = pose[0] - self._last_pose[0]
        dy = pose[1] - self._last_pose[1]
        self.total_distance_traveled += math.sqrt(dx*dx + dy*dy)
        self._last_pose = fused_pose
        
        # Calculate distance and heading error
        self.distance_to_goal = self._distance_to_goal(pose[0], pose[1])
        self.heading_error = self._angle_to_goal(fused_pose)
        
        # State machine
        if self.state == GoToGoalState.TURNING:
            return self._handle_turning(fused_pose)
        elif self.state == GoToGoalState.DRIVING:
            return self._handle_driving(fused_pose)
        elif self.state == GoToGoalState.FINAL_APPROACH:
            return self._handle_final_approach(fused_pose)
        else:
            return (0.0, 0.0)
    
    def _handle_turning(self, pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """Handle turning state with PID control"""
        # Check timeout
        if time.time() - self._turn_start_time > self.config.turn_timeout:
            logger.warning(f"Turn timeout! Error: {math.degrees(self.heading_error):.1f} deg")
            # Force proceed if close enough
            if abs(self.heading_error) < math.radians(30):
                self.state = GoToGoalState.DRIVING
                self.drive_pid.reset()
                return self._handle_driving(pose)
            else:
                self._turn_start_time = time.time()
                logger.info("Retrying turn...")
                self.turn_pid.reset()
        
        # Check if we're facing the goal
        abs_error = abs(self.heading_error)
        if abs_error < self.config.turn_tolerance:
            logger.info(f"Facing goal! Error: {math.degrees(abs_error):.1f} deg")
            self.state = GoToGoalState.DRIVING
            self.drive_pid.reset()
            return self._handle_driving(pose)
        
        # Use PID for turning
        dt = 1.0 / self.config.control_rate
        w = self.turn_pid.update(self.heading_error, dt)
        
        # Boost turning for large errors
        if abs_error > math.radians(90):
            w = math.copysign(self.config.max_angular_speed, w)
        elif abs_error > math.radians(45):
            w = max(-self.config.max_angular_speed * 0.8, 
                   min(w, self.config.max_angular_speed * 0.8))
        
        # Zero linear velocity (turn in place)
        v = 0.0
        
        self._last_command = (v, w)
        return (v, w)
    
    def _handle_driving(self, pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """Handle driving state with PID heading correction"""
        # Check if we need to re-turn
        if abs(self.heading_error) > math.radians(20):
            logger.info(f"Re-turning: error={math.degrees(self.heading_error):.1f} deg")
            self.state = GoToGoalState.TURNING
            self._turn_start_time = time.time()
            self.turn_pid.reset()
            return self._handle_turning(pose)
        
        # Check for final approach
        if self.distance_to_goal < self.config.approach_distance:
            self.state = GoToGoalState.FINAL_APPROACH
            return self._handle_final_approach(pose)
        
        # Calculate linear velocity with slowdown
        if self.distance_to_goal < self.config.slowdown_distance:
            # Smooth slowdown
            factor = (self.distance_to_goal / self.config.slowdown_distance) ** 1.5
            factor = max(0.15, min(1.0, factor))
            v = self.config.max_linear_speed * factor
        else:
            v = self.config.max_linear_speed
        
        # Heading correction while driving (PID)
        dt = 1.0 / self.config.control_rate
        w = self.drive_pid.update(self.heading_error, dt)
        
        # Ensure minimum speed to move
        v = max(self.config.min_linear_speed, v)
        
        self._last_command = (v, w)
        return (v, w)
    
    def _handle_final_approach(self, pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """Handle final approach with very slow speed"""
        # Very slow speed
        speed_factor = max(0.2, self.distance_to_goal / self.config.approach_distance)
        v = self.config.max_linear_speed * 0.3 * speed_factor
        v = max(self.config.min_linear_speed, min(v, self.config.max_linear_speed * 0.3))
        
        # Fine heading correction (gentle)
        dt = 1.0 / self.config.control_rate
        w = self.drive_pid.update(self.heading_error, dt) * 0.5
        
        # Limit angular velocity
        max_w = 0.2
        w = max(-max_w, min(w, max_w))
        
        # If very close, go straight
        if self.distance_to_goal < 0.08:
            w = 0.0
        
        # Check if goal reached
        if self.distance_to_goal < self.config.position_tolerance:
            self.state = GoToGoalState.GOAL_REACHED
            return (0.0, 0.0)
        
        self._last_command = (v, w)
        return (v, w)
    
    def _fuse_heading(self, odom_heading: float, imu_heading: Optional[float]) -> float:
        """Fuse odometry and IMU heading"""
        if not self.config.use_imu or imu_heading is None:
            return odom_heading
        
        # Calculate discrepancy
        diff = self._normalize_angle(odom_heading - imu_heading)
        
        # If discrepancy is too large, trust odometry
        if abs(diff) > math.radians(90):
            logger.debug(f"Large heading discrepancy: {math.degrees(diff):.1f} deg")
            return odom_heading
        
        # Weighted average (IMU primary)
        w_odom = 1.0 - self.config.imu_weight
        fused = imu_heading + w_odom * diff
        
        return self._normalize_angle(fused)
    
    def _check_stuck(self, state) -> bool:
        """Check if robot is stuck"""
        if self.state in [GoToGoalState.GOAL_REACHED, GoToGoalState.IDLE]:
            return False
        
        # Check if distance hasn't decreased
        if self.distance_to_goal < self._prev_distance - 0.005:
            self._prev_distance = self.distance_to_goal
            self._stuck_start_time = time.time()
            return False
        
        # Check if stuck for too long
        if time.time() - self._stuck_start_time > self.config.stuck_threshold:
            # But only if we're not already turning
            if self.state != GoToGoalState.TURNING:
                return True
        
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
    
    def _log_status(self, state, v: float, w: float):
        """Log current status"""
        logger.info(
            f"dist={self.distance_to_goal:.3f}m, "
            f"heading_err={math.degrees(self.heading_error):.1f} deg, "
            f"state={self.state.value}, "
            f"v={v:.3f}, w={w:.3f}, "
            f"pos=({state.x:.2f}, {state.y:.2f})"
        )
    
    def _get_status_dict(self, state, v: float, w: float) -> dict:
        """Get status as dictionary for callback"""
        return {
            'state': self.state.value,
            'goal_x': self.goal[0],
            'goal_y': self.goal[1],
            'x': state.x,
            'y': state.y,
            'heading': state.heading,
            'distance_to_goal': self.distance_to_goal,
            'heading_error': self.heading_error,
            'heading_error_deg': math.degrees(self.heading_error),
            'velocity': (v, w),
            'elapsed_time': self.elapsed_time,
            'total_distance': self.total_distance_traveled,
            'goal_reached': self.state == GoToGoalState.GOAL_REACHED
        }
    
    def stop(self):
        """Stop the robot"""
        self.robot.stop()
        self.state = GoToGoalState.STOPPED
        self.turn_pid.reset()
        self.drive_pid.reset()
        logger.info("Stopped")
    
    def resume(self):
        """Resume navigation"""
        if self.state == GoToGoalState.STOPPED:
            self.state = GoToGoalState.TURNING
            self._turn_start_time = time.time()
            self._stuck_start_time = time.time()
            logger.info("Resumed")
    
    def get_status(self) -> dict:
        """Get current status"""
        state = self.robot.get_state()
        return self._get_status_dict(state, self._last_command[0], self._last_command[1])


# Simple test script
def main():
    """Test the GoToGoal controller"""
    import argparse
    
    parser = argparse.ArgumentParser(description="GoToGoal Test")
    parser.add_argument('--goal', type=float, nargs=2, required=True)
    parser.add_argument('--port', type=str, help='Serial port')
    parser.add_argument('--simulate', action='store_true')
    parser.add_argument('--speed', type=float, default=0.25)
    parser.add_argument('--timeout', type=float, default=60.0)
    
    args = parser.parse_args()
    
    if args.simulate:
        from jidenna import RobotAPI
        robot = RobotAPI()
        # Mock robot for simulation
        class MockRobot:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.heading = 0.0
                self.last_time = time.time()
            
            def get_state(self):
                class State:
                    def __init__(self):
                        self.x = 0.0
                        self.y = 0.0
                        self.heading = 0.0
                        self.is_imu_valid = lambda: False
                        self.imu_heading = 0.0
                        self.is_valid = lambda: True
                return State()
            
            def set_velocity(self, v, w):
                pass
            
            def stop(self):
                pass
        
        robot = MockRobot()
    else:
        if not args.port:
            print("Error: --port required for real robot")
            return
        from jidenna import RobotAPI
        robot = RobotAPI(args.port)
        if not robot.connect():
            print(f"Failed to connect to {args.port}")
            return
    
    # Create config
    config = GoToGoalConfig()
    config.max_linear_speed = args.speed
    config.max_time = args.timeout
    
    # Create controller
    gtg = GoToGoal(robot, config)
    
    # Define callback
    def status_callback(status):
        pass  # Optional: print or log status
    
    # Go to goal
    success = gtg.go_to(args.goal[0], args.goal[1], callback=status_callback)
    
    result = "YES" if success else "NO"
    print(f"\nGoal: ({args.goal[0]}, {args.goal[1]}) - {result}")
    
    if not args.simulate:
        robot.disconnect()


if __name__ == "__main__":
    main()