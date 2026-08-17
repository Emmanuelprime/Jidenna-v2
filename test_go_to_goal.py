#!/usr/bin/env python3
"""
Test script for Local Planner with Pure Pursuit - Go to Goal

This script allows you to set a goal (x, y) and the robot will navigate to it
using the local planner with pure pursuit path tracking.

Usage:
    python test_go_to_goal.py [--goal X Y] [--simulate] [--port PORT]

Examples:
    # Simulate navigation to goal (1.0, 1.0)
    python test_go_to_goal.py --goal 1.0 1.0 --simulate
    
    # Real robot navigation to goal (2.0, 3.0)
    python test_go_to_goal.py --goal 2.0 3.0 --port /dev/ttyUSB0
"""

import sys
import time
import math
import argparse
import logging
import threading
from typing import Tuple, Optional
from dataclasses import dataclass

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Import robot modules
try:
    from jidenna.robot import RobotAPI
    from jidenna.state import RobotState
    from navigation.local_planner import LocalPlanner, PlannerConfig, PlannerState
    from navigation.path import Path
    from jidenna.sensors import UltrasonicAPI
except ImportError as e:
    logger.error(f"Failed to import robot modules: {e}")
    logger.error("Make sure you're running from the correct directory")
    sys.exit(1)


@dataclass
class NavigationStatus:
    """Navigation status tracking"""
    goal_x: float = 0.0
    goal_y: float = 0.0
    current_x: float = 0.0
    current_y: float = 0.0
    distance_to_goal: float = float('inf')
    state: str = "IDLE"
    velocity: Tuple[float, float] = (0.0, 0.0)
    elapsed_time: float = 0.0
    is_navigating: bool = False
    is_goal_reached: bool = False
    
    def __str__(self) -> str:
        return (f"Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) | "
                f"Position: ({self.current_x:.2f}, {self.current_y:.2f}) | "
                f"Distance: {self.distance_to_goal:.3f}m | "
                f"State: {self.state} | "
                f"v: {self.velocity[0]:.3f}, w: {self.velocity[1]:.3f}")


class GoToGoal:
    """
    Go to goal controller using Local Planner and Pure Pursuit
    """
    
    def __init__(self, 
                 robot_api: Optional[RobotAPI] = None,
                 ultrasonic_api: Optional[UltrasonicAPI] = None,
                 simulate: bool = False,
                 planner_config: Optional[PlannerConfig] = None):
        """
        Initialize Go to Goal controller
        
        Args:
            robot_api: RobotAPI instance for real robot
            ultrasonic_api: UltrasonicAPI instance for sensors
            simulate: If True, run in simulation mode
            planner_config: Custom planner configuration
        """
        self.robot_api = robot_api
        self.ultrasonic_api = ultrasonic_api
        self.simulate = simulate
        
        # Create planner with configuration
        if planner_config is None:
            # Tuned for smooth navigation
            config = PlannerConfig()
            config.lookahead_distance = 0.4
            config.lookahead_min = 0.25
            config.lookahead_max = 0.8
            config.max_linear_velocity = 0.3
            config.max_angular_velocity = 0.8
            config.max_linear_acceleration = 0.25
            config.max_angular_acceleration = 0.4
            config.position_tolerance = 0.12
            config.heading_tolerance = math.radians(20)
            config.goal_slowdown_distance = 0.4
            config.final_approach_distance = 0.25
            config.position_correction_gain = 2.5
            config.control_frequency = 20.0
            config.control_dt = 0.05
        else:
            config = planner_config
            
        self.planner = LocalPlanner(config)
        
        # Navigation state
        self.status = NavigationStatus()
        self._running = False
        self._control_thread = None
        self._lock = threading.Lock()
        
        # For simulation
        self.sim_pose = (0.0, 0.0, 0.0)
        self.sim_time = 0.0
        
        # Path generated from goal
        self.path_points = []
        
        logger.info("GoToGoal initialized")
        if simulate:
            logger.info("Running in SIMULATION mode")
        else:
            logger.info("Running in REAL ROBOT mode")
    
    def set_goal(self, x: float, y: float) -> bool:
        """
        Set goal position and start navigation
        
        Args:
            x: Goal X position in meters
            y: Goal Y position in meters
            
        Returns:
            True if goal set successfully
        """
        with self._lock:
            self.status.goal_x = x
            self.status.goal_y = y
            self.status.is_navigating = True
            self.status.is_goal_reached = False
            self.status.elapsed_time = 0.0
            
            # Generate path from current position to goal
            if self.simulate:
                current_pos = (self.sim_pose[0], self.sim_pose[1])
            else:
                state = self.robot_api.get_state()
                current_pos = (state.x, state.y)
            
            # Create a simple straight-line path to goal with some waypoints
            self.path_points = self._generate_path(current_pos, (x, y))
            
            # Set path in planner
            self.planner.set_path(self.path_points)
            
            logger.info(f"Goal set to ({x:.2f}, {y:.2f})")
            logger.info(f"Path has {len(self.path_points)} waypoints")
            
            return True
    
    def _generate_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> list:
        """
        Generate a path from start to goal with waypoints
        
        Args:
            start: (x, y) start position
            goal: (x, y) goal position
            
        Returns:
            List of (x, y) waypoints
        """
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.01:
            return [start, goal]
        
        # Generate waypoints along straight line
        num_points = max(2, int(distance / 0.15))  # One waypoint every 15cm
        waypoints = []
        
        for i in range(num_points + 1):
            t = i / num_points
            x = start[0] + t * dx
            y = start[1] + t * dy
            waypoints.append((x, y))
        
        # Add a slight overshoot to ensure we pass through goal
        # This helps with final approach
        if distance > 0.5:
            overshoot = 0.05  # 5cm overshoot
            if overshoot < distance * 0.1:
                overshoot_x = goal[0] + overshoot * dx / distance
                overshoot_y = goal[1] + overshoot * dy / distance
                waypoints.append((overshoot_x, overshoot_y))
        
        return waypoints
    
    def start(self):
        """Start the control loop"""
        if self._running:
            logger.warning("Control loop already running")
            return
        
        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        logger.info("Control loop started")
    
    def stop(self):
        """Stop the control loop and robot"""
        self._running = False
        if self._control_thread:
            self._control_thread.join(timeout=2.0)
        
        # Send stop command
        if not self.simulate and self.robot_api:
            self.robot_api.stop()
        
        self.planner.stop()
        logger.info("Control loop stopped")
    
    def _control_loop(self):
        """Main control loop"""
        loop_rate = 20.0  # Hz
        dt = 1.0 / loop_rate
        last_time = time.time()
        
        logger.info(f"Control loop running at {loop_rate} Hz")
        
        while self._running:
            try:
                current_time = time.time()
                dt_actual = current_time - last_time
                last_time = current_time
                
                # Get robot pose
                if self.simulate:
                    pose = self.sim_pose
                    imu_heading = pose[2]  # Use heading from sim
                else:
                    # Get state from robot
                    state = self.robot_api.get_state()
                    pose = (state.x, state.y, state.heading)
                    imu_heading = state.imu_heading if state.is_imu_valid() else None
                
                # Update planner
                v, w = self.planner.update(pose, imu_heading)
                
                # Get planner state
                planner_state = self.planner.get_state()
                
                # Update status
                with self._lock:
                    self.status.current_x = pose[0]
                    self.status.current_y = pose[1]
                    self.status.distance_to_goal = self.planner.distance_to_goal
                    self.status.state = planner_state.value
                    self.status.velocity = (v, w)
                    self.status.elapsed_time += dt_actual
                    self.status.is_goal_reached = (planner_state == PlannerState.GOAL_REACHED)
                
                # Send velocity command
                if not self.simulate:
                    if self.robot_api:
                        self.robot_api.set_velocity(v, w)
                    else:
                        logger.error("No robot API available")
                        break
                else:
                    # Simulation: update pose
                    self._update_simulation(v, w, dt_actual)
                
                # Log status periodically
                if current_time - getattr(self, '_last_log_time', 0) > 1.0:
                    self._last_log_time = current_time
                    logger.info(str(self.status))
                
                # Check if goal reached
                if self.status.is_goal_reached:
                    logger.info(f"🎯 Goal reached! Position: ({pose[0]:.2f}, {pose[1]:.2f})")
                    if not self.simulate:
                        self.robot_api.stop()
                    self._running = False
                    break
                
                # Sleep to maintain loop rate
                sleep_time = (1.0 / loop_rate) - dt_actual
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                logger.error(f"Control loop error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    def _update_simulation(self, v: float, w: float, dt: float):
        """
        Update simulated robot pose
        
        Args:
            v: Linear velocity
            w: Angular velocity
            dt: Time step
        """
        x, y, heading = self.sim_pose
        
        # Update heading
        heading += w * dt
        heading = self._normalize_angle(heading)
        
        # Update position
        x += v * math.cos(heading) * dt
        y += v * math.sin(heading) * dt
        
        self.sim_pose = (x, y, heading)
    
    def get_status(self) -> NavigationStatus:
        """Get current navigation status"""
        with self._lock:
            return self.status
    
    def wait_for_goal(self, timeout: float = 60.0) -> bool:
        """
        Wait for goal to be reached
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if goal reached, False if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.status.is_goal_reached:
                return True
            time.sleep(0.1)
        return False
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="Go to Goal using Local Planner and Pure Pursuit"
    )
    parser.add_argument(
        "--goal", type=float, nargs=2, default=[1.0, 1.0],
        help="Goal position (x y) in meters"
    )
    parser.add_argument(
        "--simulate", action="store_true",
        help="Run in simulation mode (no robot)"
    )
    parser.add_argument(
        "--port", type=str, default=None,
        help="Serial port for robot (e.g., /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Baud rate for serial communication"
    )
    parser.add_argument(
        "--ultrasonic-port", type=str, default=None,
        help="Serial port for ultrasonic sensors"
    )
    parser.add_argument(
        "--timeout", type=float, default=60.0,
        help="Timeout for navigation in seconds"
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="Enable debug logging"
    )
    
    args = parser.parse_args()
    
    # Set logging level
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Initialize robot API (if not in simulation)
    robot_api = None
    ultrasonic_api = None
    
    if not args.simulate:
        # Connect to robot
        robot_api = RobotAPI(args.port, args.baud)
        if not robot_api.connect():
            logger.error("Failed to connect to robot")
            return 1
        
        logger.info(f"Connected to robot on {args.port or 'auto'}")
        
        # Connect to ultrasonic sensors if port specified
        if args.ultrasonic_port:
            ultrasonic_api = UltrasonicAPI(args.ultrasonic_port)
            if ultrasonic_api.connect():
                logger.info(f"Connected to ultrasonic sensors on {args.ultrasonic_port}")
            else:
                logger.warning("Failed to connect to ultrasonic sensors")
    
    try:
        # Create GoToGoal controller
        controller = GoToGoal(
            robot_api=robot_api,
            ultrasonic_api=ultrasonic_api,
            simulate=args.simulate
        )
        
        # Set goal
        goal_x, goal_y = args.goal
        if not controller.set_goal(goal_x, goal_y):
            logger.error("Failed to set goal")
            return 1
        
        # Start navigation
        logger.info("Starting navigation...")
        controller.start()
        
        # Wait for goal
        if controller.wait_for_goal(args.timeout):
            logger.info("✅ Navigation completed successfully!")
        else:
            logger.warning(f"⏱️ Navigation timeout after {args.timeout}s")
        
        # Stop controller
        controller.stop()
        
        # Print final status
        status = controller.get_status()
        print("\n=== Navigation Summary ===")
        print(f"Goal: ({status.goal_x:.2f}, {status.goal_y:.2f})")
        print(f"Final position: ({status.current_x:.2f}, {status.current_y:.2f})")
        print(f"Final distance: {status.distance_to_goal:.3f}m")
        print(f"Time elapsed: {status.elapsed_time:.1f}s")
        print(f"Goal reached: {status.is_goal_reached}")
        
    except KeyboardInterrupt:
        logger.info("\nInterrupted by user")
        if 'controller' in locals():
            controller.stop()
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # Clean up
        if robot_api:
            robot_api.stop()
            robot_api.disconnect()
            logger.info("Robot disconnected")
        if ultrasonic_api:
            ultrasonic_api.disconnect()
            logger.info("Ultrasonic sensors disconnected")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())