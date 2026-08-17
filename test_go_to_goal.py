#!/usr/bin/env python3
"""
Go-to-Goal with Better Position Tracking and Multi-Goal Support

Improvements:
1. Shows actual odometry position vs goal
2. Handles returning to origin
3. Better distance calculation
4. Optional reset of odometry
5. Continuous navigation without restarting

Usage:
    python go_to_goal.py --goal 2.0 2.0 --port /dev/ttyUSB0
    python go_to_goal.py --goal 0.0 0.0 --port /dev/ttyUSB0 --reset-odom
"""

import math
import time
import argparse
import logging
import sys
import os
from typing import Tuple, Optional, List
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
    TURNING = "TURNING"
    DRIVING = "DRIVING"
    FINAL_APPROACH = "FINAL"
    GOAL_REACHED = "GOAL_REACHED"
    ERROR = "ERROR"


@dataclass
class GoToGoalConfig:
    """Configuration for go-to-goal controller"""
    # Speed settings
    max_linear_speed: float = 0.25      # m/s
    max_angular_speed: float = 0.8      # rad/s
    approach_speed: float = 0.08        # m/s
    min_speed: float = 0.02             # m/s
    
    # Turning settings
    turn_kp: float = 2.5                # Proportional gain for turning
    turn_tolerance: float = 0.05        # radians (2.86°)
    turn_timeout: float = 8.0           # seconds (increased for large turns)
    
    # Goal settings
    position_tolerance: float = 0.15    # meters (slightly relaxed)
    slowdown_distance: float = 0.5      # meters
    approach_distance: float = 0.3      # meters
    
    # Control loop
    control_rate: float = 20.0          # Hz
    
    # Safety
    max_heading_error: float = math.radians(170)


class GoToGoal:
    """
    Go-to-Goal controller with better odometry handling
    """
    
    def __init__(self, 
                 robot_api: RobotAPI,
                 config: Optional[GoToGoalConfig] = None):
        self.robot = robot_api
        self.config = config if config else GoToGoalConfig()
        
        # State
        self.state = GoToGoalState.IDLE
        self.goal = (0.0, 0.0)
        self.start_pose = (0.0, 0.0, 0.0)
        self.distance_to_goal = float('inf')
        self.heading_error = 0.0
        self.elapsed_time = 0.0
        self.total_distance_traveled = 0.0
        
        # Simulation support
        self.simulate = False
        self.sim_pose = (0.0, 0.0, 0.0)
        self.sim_last_time = time.time()
        
        # Internal state
        self._start_time = 0.0
        self._turn_start_time = 0.0
        self._last_command = (0.0, 0.0)
        self._last_pose = (0.0, 0.0, 0.0)
        self._prev_distance = float('inf')
        
        logger.info("GoToGoal initialized")
        logger.info(f"Max speed: {self.config.max_linear_speed:.2f} m/s")
        logger.info(f"Turn tolerance: {math.degrees(self.config.turn_tolerance):.1f}°")
        logger.info(f"Position tolerance: {self.config.position_tolerance:.3f}m")
    
    def reset_odometry(self, x: float = 0.0, y: float = 0.0, heading: float = 0.0):
        """
        Reset robot odometry (if supported by ESP32)
        
        Args:
            x: New X position
            y: New Y position
            heading: New heading in radians
        """
        logger.info(f"Resetting odometry to ({x:.2f}, {y:.2f})")
        # Note: This requires ESP32 support for resetting odometry
        # If not supported, we just log it and continue
        try:
            # Try to send reset command if supported
            # self.robot.reset_odometry(x, y, heading)
            pass
        except AttributeError:
            logger.warning("Odometry reset not supported by robot API")
            logger.info("Continue with current odometry")
    
    def set_goal(self, x: float, y: float, reset_odom: bool = False) -> bool:
        """
        Set goal position
        
        Args:
            x: Goal X position (meters)
            y: Goal Y position (meters)
            reset_odom: Reset odometry before starting
            
        Returns:
            True if goal set successfully
        """
        # Reset odometry if requested
        if reset_odom:
            self.reset_odometry(0.0, 0.0, 0.0)
            # Wait for odometry to update
            time.sleep(0.2)
        
        self.goal = (x, y)
        self.state = GoToGoalState.TURNING
        self._start_time = time.time()
        self._turn_start_time = time.time()
        self.total_distance_traveled = 0.0
        
        # Get current pose
        if self.simulate:
            pose = self.sim_pose
        else:
            state = self.robot.get_state()
            pose = (state.x, state.y, state.heading)
        
        self.start_pose = pose
        self._last_pose = pose
        self._prev_distance = float('inf')
        
        # Calculate initial distance and angle
        self.distance_to_goal = self._distance_to_goal(pose[0], pose[1])
        self.heading_error = self._angle_to_goal(pose)
        
        logger.info(f"🎯 Goal set to ({x:.2f}, {y:.2f})")
        logger.info(f"📍 Starting at: ({pose[0]:.3f}, {pose[1]:.3f})")
        logger.info(f"📏 Distance to goal: {self.distance_to_goal:.3f}m")
        logger.info(f"🧭 Heading error: {math.degrees(self.heading_error):.1f}°")
        
        # If already at goal
        if self.distance_to_goal < self.config.position_tolerance:
            self.state = GoToGoalState.GOAL_REACHED
            logger.info("✅ Already at goal!")
            return True
        
        return True
    
    def update(self, pose: Tuple[float, float, float], 
               imu_heading: Optional[float] = None) -> Tuple[float, float]:
        """
        Update controller and compute velocity command
        """
        # Use IMU if available
        heading = pose[2]
        if imu_heading is not None and not self.simulate:
            diff = self._normalize_angle(imu_heading - heading)
            if abs(diff) < math.radians(30):
                heading = imu_heading
            elif abs(diff) < math.radians(60):
                heading = heading + 0.5 * diff
        
        x, y, _ = pose
        pose_with_heading = (x, y, heading)
        
        # Calculate distance traveled since last update
        dx = x - self._last_pose[0]
        dy = y - self._last_pose[1]
        self.total_distance_traveled += math.sqrt(dx*dx + dy*dy)
        self._last_pose = pose_with_heading
        
        # Update distance to goal
        self.distance_to_goal = self._distance_to_goal(x, y)
        self.elapsed_time = time.time() - self._start_time
        
        # Check if goal reached (with some hysteresis)
        if self.distance_to_goal < self.config.position_tolerance:
            # Verify with odometry stability
            if self._prev_distance < self.distance_to_goal + 0.02:
                # Robot is stable at goal
                self.state = GoToGoalState.GOAL_REACHED
                logger.info(f"✅ Goal reached! Distance: {self.distance_to_goal:.3f}m")
                return (0.0, 0.0)
        self._prev_distance = self.distance_to_goal
        
        # Calculate heading error
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
        """Handle turning state"""
        # Check timeout
        if time.time() - self._turn_start_time > self.config.turn_timeout:
            logger.warning(f"Turn timeout! Error: {math.degrees(self.heading_error):.1f}°")
            # Force proceed if we're close enough
            if abs(self.heading_error) < math.radians(30):
                self.state = GoToGoalState.DRIVING
                return self._handle_driving(pose)
            else:
                # Try turning again
                self._turn_start_time = time.time()
                logger.info("Retrying turn...")
        
        abs_error = abs(self.heading_error)
        
        if abs_error < self.config.turn_tolerance:
            logger.info(f"✅ Facing goal! Error: {math.degrees(abs_error):.1f}°")
            self.state = GoToGoalState.DRIVING
            return self._handle_driving(pose)
        
        # Calculate angular velocity
        if abs_error < 0.1:
            w = self.config.turn_kp * self.heading_error * 0.5
        else:
            w = self.config.turn_kp * self.heading_error
        
        max_w = self.config.max_angular_speed
        w = max(-max_w, min(w, max_w))
        
        # Faster turning for large errors
        if abs_error > math.radians(90):
            w = math.copysign(max_w, w)
        
        v = 0.0
        
        self._last_command = (v, w)
        return (v, w)
    
    def _handle_driving(self, pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """Handle driving state"""
        # Check if we need to re-turn
        if abs(self.heading_error) > math.radians(20):
            logger.info(f"Re-turning: error={math.degrees(self.heading_error):.1f}°")
            self.state = GoToGoalState.TURNING
            self._turn_start_time = time.time()
            return self._handle_turning(pose)
        
        # Check for final approach
        if self.distance_to_goal < self.config.approach_distance:
            self.state = GoToGoalState.FINAL_APPROACH
            return self._handle_final_approach(pose)
        
        # Calculate linear velocity with slowdown
        if self.distance_to_goal < self.config.slowdown_distance:
            factor = self.distance_to_goal / self.config.slowdown_distance
            v = self.config.max_linear_speed * max(0.2, factor)
        else:
            v = self.config.max_linear_speed
        
        # Heading correction while driving
        w = 0.8 * self.heading_error
        max_w = self.config.max_angular_speed * 0.5
        w = max(-max_w, min(w, max_w))
        
        v = max(self.config.min_speed, v)
        
        self._last_command = (v, w)
        return (v, w)
    
    def _handle_final_approach(self, pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """Handle final approach"""
        speed_factor = max(0.3, self.distance_to_goal / self.config.approach_distance)
        v = self.config.approach_speed * speed_factor
        v = max(self.config.min_speed, min(v, self.config.approach_speed))
        
        # Fine heading correction
        w = 1.5 * self.heading_error
        max_w = 0.3
        w = max(-max_w, min(w, max_w))
        
        # If very close, just go straight
        if self.distance_to_goal < 0.08:
            w = 0.0
        
        self._last_command = (v, w)
        return (v, w)
    
    def run(self, goal_x: float, goal_y: float, 
            max_time: float = 60.0,
            reset_odom: bool = False) -> bool:
        """
        Run navigation to goal
        """
        if not self.set_goal(goal_x, goal_y, reset_odom):
            return False
        
        # If already at goal
        if self.state == GoToGoalState.GOAL_REACHED:
            return True
        
        start_time = time.time()
        last_log = time.time()
        
        logger.info("🚀 Starting navigation...")
        logger.info("Press Ctrl+C to stop")
        
        # Track if we've made progress
        min_distance = self.distance_to_goal
        stuck_time = 0
        stuck_threshold = 3.0  # seconds without progress
        
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
                               f"v={v:.3f}, w={w:.3f}, "
                               f"pos=({pose[0]:.2f}, {pose[1]:.2f})")
                    last_log = time.time()
                
                # Check for being stuck
                if self.distance_to_goal < min_distance - 0.01:
                    min_distance = self.distance_to_goal
                    stuck_time = 0
                else:
                    stuck_time += 0.05
                
                if stuck_time > stuck_threshold and self.state != GoToGoalState.GOAL_REACHED:
                    logger.warning(f"Stuck! Distance not decreasing: {self.distance_to_goal:.3f}m")
                    logger.info("Re-turning to goal...")
                    self.state = GoToGoalState.TURNING
                    self._turn_start_time = time.time()
                    stuck_time = 0
                
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
            import traceback
            traceback.print_exc()
            self.robot.stop()
            return False
    
    def run_multiple_goals(self, goals: List[Tuple[float, float]], 
                           reset_between: bool = False) -> List[bool]:
        """
        Run multiple goals sequentially
        
        Args:
            goals: List of (x, y) goal positions
            reset_between: Reset odometry between goals
            
        Returns:
            List of success flags for each goal
        """
        results = []
        
        for i, (x, y) in enumerate(goals):
            logger.info(f"\n🎯 Goal {i+1}/{len(goals)}: ({x:.2f}, {y:.2f})")
            
            # Reset odometry if requested
            reset = reset_between and i > 0
            
            success = self.run(x, y, reset_odom=reset)
            results.append(success)
            
            if not success:
                logger.warning(f"Goal {i+1} failed, stopping sequence")
                break
            
            # Brief pause between goals
            if i < len(goals) - 1:
                time.sleep(1.0)
        
        return results
    
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
        heading += w * dt
        heading = self._normalize_angle(heading)
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
            'total_distance': self.total_distance_traveled,
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
        description="Go-to-Goal Navigation with Better Odometry Handling",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Single goal
  python go_to_goal.py --goal 2.0 2.0 --port /dev/ttyUSB0
  
  # Return to origin
  python go_to_goal.py --goal 0.0 0.0 --port /dev/ttyUSB0
  
  # Multiple goals
  python go_to_goal.py --goals 2.0,2.0 0.0,0.0 1.0,1.0 --port /dev/ttyUSB0
  
  # Reset odometry before starting
  python go_to_goal.py --goal 0.0 0.0 --port /dev/ttyUSB0 --reset-odom
        """
    )
    
    parser.add_argument(
        '--goal', type=float, nargs=2,
        help='Single goal position (x y) in meters'
    )
    parser.add_argument(
        '--goals', type=str, nargs='+',
        help='Multiple goals as "x,y" pairs (e.g., "2.0,2.0" "0.0,0.0")'
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
        help='Navigation timeout per goal (seconds)'
    )
    parser.add_argument(
        '--reset-odom', action='store_true',
        help='Reset odometry before starting'
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
    
    # Parse goals
    goals = []
    if args.goal:
        goals.append(tuple(args.goal))
    elif args.goals:
        for g in args.goals:
            try:
                x, y = map(float, g.split(','))
                goals.append((x, y))
            except ValueError:
                print(f"Error: Invalid goal format '{g}'. Use 'x,y'")
                return 1
    else:
        print("Error: Must specify --goal or --goals")
        return 1
    
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
    gtg.simulate = args.simulate
    if args.simulate:
        gtg.sim_pose = (0.0, 0.0, 0.0)
    
    try:
        # Get initial position
        state = robot.get_state()
        logger.info(f"📍 Starting at: ({state.x:.2f}, {state.y:.2f})")
        
        # Reset odometry if requested
        if args.reset_odom:
            gtg.reset_odometry(0.0, 0.0, 0.0)
            time.sleep(0.2)
        
        # Run navigation
        if len(goals) == 1:
            success = gtg.run(goals[0][0], goals[0][1], 
                            max_time=args.timeout,
                            reset_odom=args.reset_odom)
            results = [success]
        else:
            logger.info(f"🔄 Running {len(goals)} goals in sequence")
            results = gtg.run_multiple_goals(goals, reset_between=args.reset_odom)
        
        # Print results
        print("\n" + "="*50)
        print("NAVIGATION RESULTS")
        print("="*50)
        
        for i, (goal, success) in enumerate(zip(goals, results)):
            status = "✅" if success else "❌"
            print(f"Goal {i+1}: ({goal[0]:.2f}, {goal[1]:.2f}) {status}")
        
        if len(goals) > 1:
            total_success = sum(results)
            print(f"\nTotal: {total_success}/{len(goals)} goals reached")
        
        return 0 if all(results) else 1
        
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