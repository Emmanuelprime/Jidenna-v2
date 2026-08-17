#!/usr/bin/env python3
"""
Simple Go-to-Goal Test Script

Allows you to set a single goal (x, y) and navigate there using the local planner.
Uses the existing PathGenerator to create a straight-line path to the goal.

Usage:
    python go_to_goal.py --goal 1.0 1.0 --simulate
    python go_to_goal.py --goal 2.0 3.0 --port /dev/ttyUSB0
    python go_to_goal.py --goal 1.5 0.5 --port COM3 --no-imu
"""

import math
import time
import argparse
import logging
import sys
import os
from typing import Tuple, List, Optional
from dataclasses import dataclass

# Add parent directory to path if needed
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from jidenna import RobotAPI, RobotState
    from navigation import LocalPlanner, PlannerConfig, PlannerState
except ImportError:
    print("Error: Could not import jidenna or navigation modules")
    print("Make sure you're running from the correct directory")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class SimplePathGenerator:
    """Simple path generator for straight-line paths to goal"""
    
    @staticmethod
    def straight_line_to_goal(start: Tuple[float, float], 
                              goal: Tuple[float, float],
                              num_points: int = 30) -> List[Tuple[float, float]]:
        """
        Generate straight-line path from start to goal
        
        Args:
            start: (x, y) starting position
            goal: (x, y) goal position
            num_points: Number of waypoints
            
        Returns:
            List of (x, y) waypoints
        """
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])
            points.append((x, y))
        return points


@dataclass
class NavigationStatus:
    """Current navigation status"""
    goal: Tuple[float, float] = (0.0, 0.0)
    position: Tuple[float, float] = (0.0, 0.0)
    heading: float = 0.0
    distance_to_goal: float = float('inf')
    state: str = "IDLE"
    velocity: Tuple[float, float] = (0.0, 0.0)
    elapsed_time: float = 0.0
    is_goal_reached: bool = False
    
    def __str__(self) -> str:
        return (f"Goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f}) | "
                f"Pos: ({self.position[0]:.2f}, {self.position[1]:.2f}) | "
                f"Dist: {self.distance_to_goal:.3f}m | "
                f"State: {self.state} | "
                f"v: {self.velocity[0]:.3f}, w: {self.velocity[1]:.3f} | "
                f"Time: {self.elapsed_time:.1f}s")


class GoToGoal:
    """
    Simple Go-to-Goal controller
    
    Navigates from current position to a specified goal point
    """
    
    def __init__(self, 
                 robot_api: RobotAPI,
                 use_imu: bool = True,
                 config: Optional[PlannerConfig] = None):
        """
        Initialize Go-to-Goal controller
        
        Args:
            robot_api: Connected RobotAPI instance
            use_imu: Enable IMU fusion
            config: Custom planner configuration (optional)
        """
        self.robot = robot_api
        
        # Use provided config or create default
        if config is None:
            config = PlannerConfig(
                lookahead_distance=0.4,
                lookahead_min=0.25,
                lookahead_max=0.8,
                max_linear_velocity=0.25,  # Conservative for real robot
                max_angular_velocity=0.6,
                max_linear_acceleration=0.2,
                max_angular_acceleration=0.4,
                position_tolerance=0.12,
                heading_tolerance=math.radians(20),
                goal_slowdown_distance=0.4,
                final_approach_distance=0.25,
                final_approach_speed=0.08,
                min_approach_speed=0.03,
                use_imu_heading=use_imu,
                imu_heading_weight=0.7,
                max_heading_discrepancy=math.radians(30),
                max_curvature=3.0,
                position_correction_gain=2.0
            )
        
        self.planner = LocalPlanner(config)
        self.use_imu = use_imu
        self.status = NavigationStatus()
        self._running = False
        self._goal_set = False
        
        # Simulation mode flag
        self.simulate = False
        self.sim_pose = (0.0, 0.0, 0.0)
        
        logger.info("GoToGoal initialized")
        logger.info(f"IMU Fusion: {'ENABLED' if use_imu else 'DISABLED'}")
    
    def set_goal(self, x: float, y: float, current_pos: Optional[Tuple[float, float]] = None) -> bool:
        """
        Set goal position and generate path
        
        Args:
            x: Goal X position (meters)
            y: Goal Y position (meters)
            current_pos: Current position (if None, gets from robot)
            
        Returns:
            True if goal set successfully
        """
        if current_pos is None:
            state = self.robot.get_state()
            current_pos = (state.x, state.y)
        
        # Generate straight-line path to goal
        path = SimplePathGenerator.straight_line_to_goal(
            start=current_pos,
            goal=(x, y),
            num_points=30
        )
        
        # Set path in planner
        self.planner.set_path(path)
        
        # Update status
        self.status.goal = (x, y)
        self.status.position = current_pos
        self.status.is_goal_reached = False
        self.status.elapsed_time = 0.0
        self._goal_set = True
        
        logger.info(f"🎯 Goal set to ({x:.2f}, {y:.2f})")
        logger.info(f"📍 Starting from ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
        logger.info(f"📏 Distance to goal: {self.planner.distance_to_goal:.3f}m")
        
        return True
    
    def run(self, max_duration: float = 60.0, callback=None) -> NavigationStatus:
        """
        Run navigation to goal
        
        Args:
            max_duration: Maximum time in seconds
            callback: Optional callback function called each loop
            
        Returns:
            NavigationStatus with final results
        """
        if not self._goal_set:
            logger.error("No goal set! Call set_goal() first")
            return self.status
        
        start_time = time.time()
        self._running = True
        
        logger.info("🚀 Starting navigation...")
        logger.info("Press Ctrl+C to stop")
        
        # Get initial IMU heading if available
        imu_heading = None
        if self.use_imu:
            state = self.robot.get_state()
            if state.is_imu_valid():
                imu_heading = state.imu_heading
        
        # First update to get initial state
        state = self.robot.get_state()
        pose = (state.x, state.y, state.heading)
        self.planner.update(pose, imu_heading)
        
        last_log_time = time.time()
        
        try:
            while self._running:
                current_time = time.time()
                
                # Check timeout
                if current_time - start_time > max_duration:
                    logger.warning(f"⏱️ Timeout after {max_duration}s")
                    break
                
                # Get robot state
                state = self.robot.get_state()
                if not state.is_valid():
                    logger.error("Invalid robot state!")
                    break
                
                pose = (state.x, state.y, state.heading)
                
                # Get IMU heading if available
                imu_heading = state.imu_heading if (self.use_imu and state.is_imu_valid()) else None
                
                # Update planner
                v, w = self.planner.update(pose, imu_heading)
                
                # Safety limits
                v = max(-0.3, min(v, 0.3))
                w = max(-0.8, min(w, 0.8))
                
                # Send velocity command
                self.robot.set_velocity(v, w)
                
                # Update status
                self.status.position = (state.x, state.y)
                self.status.heading = state.heading
                self.status.distance_to_goal = self.planner.distance_to_goal
                self.status.state = self.planner.get_state().value
                self.status.velocity = (v, w)
                self.status.elapsed_time = current_time - start_time
                self.status.is_goal_reached = (self.planner.get_state() == PlannerState.GOAL_REACHED)
                
                # Call callback if provided
                if callback:
                    callback(self.status)
                
                # Log progress
                if current_time - last_log_time > 1.0:
                    logger.info(f"📊 {self.status}")
                    last_log_time = current_time
                
                # Check goal reached
                if self.status.is_goal_reached:
                    logger.info("✅ GOAL REACHED!")
                    self.robot.stop()
                    break
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            logger.info("⏹️ Navigation interrupted by user")
        except Exception as e:
            logger.error(f"Navigation error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self._running = False
            self.robot.stop()
            logger.info("Robot stopped")
        
        return self.status


class SimulatedRobot:
    """Simple robot simulation for testing without hardware"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = time.time()
    
    def set_velocity(self, v: float, w: float):
        """Set velocity and update simulation"""
        current_time = time.time()
        dt = min(current_time - self.last_time, 0.1)
        self.last_time = current_time
        
        self.v = v
        self.w = w
        
        # Update pose
        self.heading += w * dt
        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt
    
    def get_state(self):
        """Get simulated robot state"""
        class SimState:
            def __init__(self, x, y, heading):
                self.x = x
                self.y = y
                self.heading = heading
                self.v = 0.0
                self.w = 0.0
                self.is_imu_valid = lambda: False
                self.imu_heading = 0.0
                self.is_valid = lambda: True
            
            def is_valid(self):
                return True
            
            def is_imu_valid(self):
                return False
        
        return SimState(self.x, self.y, self.heading)
    
    def stop(self):
        self.v = 0.0
        self.w = 0.0
    
    def disconnect(self):
        pass


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="Go-to-Goal Navigation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Simulate navigation to (1.0, 1.0)
  python go_to_goal.py --goal 1.0 1.0 --simulate
  
  # Real robot navigation to (2.0, 3.0)
  python go_to_goal.py --goal 2.0 3.0 --port /dev/ttyUSB0
  
  # Real robot with custom speed
  python go_to_goal.py --goal 1.5 0.5 --port COM3 --speed 0.3
        """
    )
    
    parser.add_argument(
        '--goal', type=float, nargs=2, required=True,
        help='Goal position (x y) in meters'
    )
    parser.add_argument(
        '--port', type=str,
        help='Serial port for robot (e.g., /dev/ttyUSB0, COM3)'
    )
    parser.add_argument(
        '--simulate', action='store_true',
        help='Run in simulation mode (no robot hardware)'
    )
    parser.add_argument(
        '--no-imu', action='store_true',
        help='Disable IMU fusion'
    )
    parser.add_argument(
        '--speed', type=float, default=0.25,
        help='Maximum linear velocity (m/s)'
    )
    parser.add_argument(
        '--timeout', type=float, default=60.0,
        help='Navigation timeout in seconds'
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
    
    # List ports if requested
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
        use_imu = False
    else:
        if not args.port:
            logger.error("--port required for real robot mode")
            return 1
        
        from jidenna import RobotAPI
        robot = RobotAPI(args.port)
        if not robot.connect():
            logger.error(f"Failed to connect to robot on {args.port}")
            return 1
        
        logger.info(f"Connected to robot on {args.port}")
        use_imu = not args.no_imu
    
    # Create custom config with user speed
    config = PlannerConfig()
    config.max_linear_velocity = args.speed
    config.max_angular_velocity = min(0.8, args.speed * 3.0)
    config.use_imu_heading = use_imu
    
    # Create GoToGoal controller
    gtg = GoToGoal(robot, use_imu=use_imu, config=config)
    
    try:
        # Get current position
        state = robot.get_state()
        current_pos = (state.x, state.y)
        
        # Set goal
        goal_x, goal_y = args.goal
        if not gtg.set_goal(goal_x, goal_y, current_pos):
            logger.error("Failed to set goal")
            return 1
        
        # Run navigation
        status = gtg.run(max_duration=args.timeout)
        
        # Print results
        print("\n" + "="*50)
        print("NAVIGATION RESULTS")
        print("="*50)
        print(f"Goal: ({status.goal[0]:.2f}, {status.goal[1]:.2f})")
        print(f"Final Position: ({status.position[0]:.2f}, {status.position[1]:.2f})")
        print(f"Final Distance: {status.distance_to_goal:.3f}m")
        print(f"Time Elapsed: {status.elapsed_time:.1f}s")
        print(f"Goal Reached: {'✅ YES' if status.is_goal_reached else '❌ NO'}")
        print(f"Final State: {status.state}")
        
        if status.is_goal_reached:
            print("\n🎯 Success! Robot reached the goal.")
            return 0
        else:
            print("\n⚠️ Goal not reached. Check robot position and logs.")
            return 1
            
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