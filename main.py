#!/usr/bin/env python3
"""
Main test program for autonomous delivery robot
Tests local planner with different path types
"""

import math
import time
import argparse
import logging
from typing import List, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Arrow

# Import our modules
from jidenna import RobotAPI
from navigation import LocalPlanner, PlannerConfig, PlannerState
from simulation.simulator import DifferentialDriveSimulator, SimulatedRobotAPI

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class PathGenerator:
    """Generates test paths"""
    
    @staticmethod
    def straight_line(start: Tuple[float, float] = (0, 0), 
                      end: Tuple[float, float] = (3, 0),
                      num_points: int = 30) -> List[Tuple[float, float]]:
        """Generate straight line path"""
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            points.append((x, y))
        return points
    
    @staticmethod
    def right_angle_path(start: Tuple[float, float] = (0, 0),
                         intermediate: Tuple[float, float] = (0, 2),
                         end: Tuple[float, float] = (3, 2),
                         num_points: int = 40) -> List[Tuple[float, float]]:
        """Generate 90-degree path"""
        points = []
        # First segment
        for i in range(num_points // 2):
            t = i / (num_points // 2 - 1)
            x = start[0] + t * (intermediate[0] - start[0])
            y = start[1] + t * (intermediate[1] - start[1])
            points.append((x, y))
        # Second segment
        for i in range(num_points // 2):
            t = i / (num_points // 2 - 1)
            x = intermediate[0] + t * (end[0] - intermediate[0])
            y = intermediate[1] + t * (end[1] - intermediate[1])
            points.append((x, y))
        return points
    
    @staticmethod
    def s_curve(start: Tuple[float, float] = (0, 0),
                end: Tuple[float, float] = (4, 2),
                amplitude: float = 0.5,
                num_points: int = 50) -> List[Tuple[float, float]]:
        """Generate S-curve path"""
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1]) + amplitude * math.sin(math.pi * t)
            points.append((x, y))
        return points
    
    @staticmethod
    def circle(center: Tuple[float, float] = (0, 0),
               radius: float = 1.5,
               num_points: int = 50) -> List[Tuple[float, float]]:
        """Generate circular path"""
        points = []
        for i in range(num_points):
            angle = (2 * math.pi * i) / (num_points - 1)
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            points.append((x, y))
        return points


class PathTester:
    """Tests local planner with different paths"""
    
    def __init__(self, use_simulation: bool = True, port: str = None):
        """Initialize tester"""
        self.use_simulation = use_simulation
        
        # Create robot API (simulated or real)
        if use_simulation:
            self.robot = SimulatedRobotAPI()
            logger.info("Using simulated robot")
        else:
            self.robot = RobotAPI(port=port)
            logger.info(f"Using real robot on {port if port else 'auto-detect'}")
        
        # Create local planner
        config = PlannerConfig(
            lookahead_distance=0.5,
            lookahead_min=0.3,
            lookahead_max=1.0,
            max_linear_velocity=0.4,
            max_angular_velocity=1.0,
            position_tolerance=0.1,
            heading_tolerance=math.radians(15)
        )
        self.planner = LocalPlanner(config)
        
        # Test data storage
        self.test_results = []
        self.current_path = None
    
    def connect(self) -> bool:
        """Connect to robot"""
        return self.robot.connect()
    
    def disconnect(self):
        """Disconnect from robot"""
        self.robot.disconnect()
    
    def run_path_test(self, path_points: List[Tuple[float, float]], 
                      path_name: str = "test",
                      max_time: float = 60.0,
                      visualize: bool = True) -> dict:
        """
        Run path following test
        
        Args:
            path_points: List of (x, y) points
            path_name: Name of the test
            max_time: Maximum test duration in seconds
            visualize: Whether to show visualization
        
        Returns:
            Test results dictionary
        """
        logger.info(f"Starting path test: {path_name}")
        
        # Set path in planner
        self.planner.set_path(path_points)
        self.current_path = path_points
        
        # Test data
        robot_positions = []
        velocity_commands = []
        lookahead_points = []
        cross_track_errors = []
        distances_to_goal = []
        planner_states = []
        timestamps = []
        
        # Initialize visualization
        if visualize:
            plt.ion()
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
            
            # Path plot
            path_x = [p[0] for p in path_points]
            path_y = [p[1] for p in path_points]
            ax1.plot(path_x, path_y, 'b-', label='Planned Path', linewidth=2)
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_title(f'Path Following: {path_name}')
            ax1.grid(True)
            ax1.axis('equal')
            
            # Error plot
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Error')
            ax2.set_title('Tracking Errors')
            ax2.grid(True)
        
        # Run test loop
        start_time = time.time()
        current_time = start_time
        
        try:
            while current_time - start_time < max_time:
                # Get current robot state
                state = self.robot.get_state()
                robot_pose = (state.x, state.y, state.heading)
                
                # Update planner
                v, w = self.planner.update(robot_pose)
                
                # Send velocity to robot
                self.robot.set_velocity(v, w)
                
                # Record data
                robot_positions.append(robot_pose)
                velocity_commands.append((v, w))
                cross_track_errors.append(self.planner.cross_track_error)
                distances_to_goal.append(self.planner.distance_to_goal)
                planner_states.append(self.planner.state)
                timestamps.append(current_time - start_time)
                
                # Get lookahead point
                debug_info = self.planner.get_debug_info()
                if debug_info['lookahead_point']:
                    lookahead_points.append(debug_info['lookahead_point'])
                else:
                    lookahead_points.append(robot_pose[:2])
                
                # Update visualization
                if visualize and len(robot_positions) % 2 == 0:
                    # Plot robot trajectory
                    traj_x = [p[0] for p in robot_positions]
                    traj_y = [p[1] for p in robot_positions]
                    
                    ax1.clear()
                    ax1.plot(path_x, path_y, 'b-', label='Planned Path', linewidth=2)
                    ax1.plot(traj_x, traj_y, 'r-', label='Actual Trajectory', linewidth=1.5)
                    
                    # Plot current robot position
                    if robot_positions:
                        current_pose = robot_positions[-1]
                        ax1.plot(current_pose[0], current_pose[1], 'go', markersize=8)
                        
                        # Plot heading arrow
                        arrow_length = 0.3
                        dx = arrow_length * math.cos(current_pose[2])
                        dy = arrow_length * math.sin(current_pose[2])
                        ax1.arrow(current_pose[0], current_pose[1], dx, dy,
                                 head_width=0.1, head_length=0.15, fc='g', ec='g')
                    
                    # Plot lookahead point
                    if lookahead_points:
                        lp = lookahead_points[-1]
                        ax1.plot(lp[0], lp[1], 'y*', markersize=12, label='Lookahead')
                    
                    ax1.legend()
                    ax1.set_xlabel('X (m)')
                    ax1.set_ylabel('Y (m)')
                    ax1.set_title(f'Path Following: {path_name}')
                    ax1.grid(True)
                    ax1.axis('equal')
                    
                    # Plot errors
                    if cross_track_errors:
                        ax2.clear()
                        ax2.plot(timestamps, cross_track_errors, 'r-', label='Cross-track Error')
                        ax2.plot(timestamps, distances_to_goal, 'b-', label='Distance to Goal')
                        ax2.legend()
                        ax2.set_xlabel('Time (s)')
                        ax2.set_ylabel('Error (m)')
                        ax2.set_title('Tracking Errors')
                        ax2.grid(True)
                    
                    plt.pause(0.01)
                
                # Check if goal reached
                if self.planner.state == PlannerState.GOAL_REACHED:
                    logger.info(f"Goal reached for {path_name}")
                    break
                
                # Check for error
                if self.planner.state == PlannerState.ERROR:
                    logger.error(f"Planner error for {path_name}")
                    break
                
                # Small delay for real robot
                if not self.use_simulation:
                    time.sleep(0.05)  # 20 Hz
                
                current_time = time.time()
        
        except KeyboardInterrupt:
            logger.info("Test interrupted by user")
        except Exception as e:
            logger.error(f"Test failed: {e}")
        finally:
            # Stop robot
            self.robot.stop()
            
            if visualize:
                plt.ioff()
                plt.show()
        
        # Compile results
        results = {
            'path_name': path_name,
            'success': self.planner.state == PlannerState.GOAL_REACHED,
            'final_state': self.planner.state,
            'duration': current_time - start_time,
            'final_pose': robot_positions[-1] if robot_positions else None,
            'final_distance_to_goal': distances_to_goal[-1] if distances_to_goal else None,
            'max_cross_track_error': max(cross_track_errors) if cross_track_errors else None,
            'avg_cross_track_error': np.mean(cross_track_errors) if cross_track_errors else None,
            'num_points': len(robot_positions)
        }
        
        self.test_results.append(results)
        logger.info(f"Test completed: {path_name}")
        logger.info(f"  Success: {results['success']}")
        logger.info(f"  Duration: {results['duration']:.2f}s")
        logger.info(f"  Final distance to goal: {results['final_distance_to_goal']:.3f}m")
        logger.info(f"  Max cross-track error: {results['max_cross_track_error']:.3f}m")
        
        return results
    
    def run_all_tests(self):
        """Run all predefined test paths"""
        logger.info("Running all path tests")
        
        # Test 1: Straight line
        path1 = PathGenerator.straight_line()
        self.run_path_test(path1, "Straight Line")
        
        # Test 2: 90-degree path
        path2 = PathGenerator.right_angle_path()
        self.run_path_test(path2, "90-Degree Path")
        
        # Test 3: S-curve
        path3 = PathGenerator.s_curve()
        self.run_path_test(path3, "S-Curve")
        
        # Test 4: Circle
        path4 = PathGenerator.circle()
        self.run_path_test(path4, "Circle")
        
        # Print summary
        logger.info("\n" + "="*50)
        logger.info("TEST SUMMARY")
        logger.info("="*50)
        for result in self.test_results:
            logger.info(f"{result['path_name']}: {'SUCCESS' if result['success'] else 'FAILED'}")
            logger.info(f"  Duration: {result['duration']:.2f}s")
            logger.info(f"  Avg CTE: {result['avg_cross_track_error']:.3f}m")
            logger.info(f"  Max CTE: {result['max_cross_track_error']:.3f}m")


def main():
    """Main test program"""
    parser = argparse.ArgumentParser(description='Test autonomous robot navigation')
    parser.add_argument('--simulate', action='store_true', help='Run in simulation mode')
    parser.add_argument('--port', type=str, help='Serial port for real robot')
    parser.add_argument('--test', type=str, choices=['straight', 'angle', 'scurve', 'circle', 'all'],
                       default='all', help='Test to run')
    
    args = parser.parse_args()
    
    # Create tester
    tester = PathTester(use_simulation=args.simulate or not args.port, port=args.port)
    
    try:
        # Connect to robot
        if not tester.connect():
            logger.error("Failed to connect to robot")
            return
        
        logger.info("Connected to robot")
        
        # Run tests
        if args.test == 'straight' or args.test == 'all':
            path = PathGenerator.straight_line()
            tester.run_path_test(path, "Straight Line")
        
        if args.test == 'angle' or args.test == 'all':
            path = PathGenerator.right_angle_path()
            tester.run_path_test(path, "90-Degree Path")
        
        if args.test == 'scurve' or args.test == 'all':
            path = PathGenerator.s_curve()
            tester.run_path_test(path, "S-Curve")
        
        if args.test == 'circle' or args.test == 'all':
            path = PathGenerator.circle()
            tester.run_path_test(path, "Circle")
        
        # Print summary if all tests
        if args.test == 'all':
            logger.info("\n" + "="*50)
            logger.info("TEST SUMMARY")
            logger.info("="*50)
            for result in tester.test_results:
                logger.info(f"{result['path_name']}: {'SUCCESS' if result['success'] else 'FAILED'}")
        
    except KeyboardInterrupt:
        logger.info("Program interrupted by user")
    finally:
        # Ensure robot is stopped and disconnected
        tester.robot.stop()
        tester.disconnect()
        logger.info("Robot disconnected")


if __name__ == "__main__":
    main()