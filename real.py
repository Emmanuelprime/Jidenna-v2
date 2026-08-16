#!/usr/bin/env python3
"""
Safe test program for real robot
Tests local planner with real ESP32 hardware
Uses IMU data for improved heading estimation
"""

import math
import time
import argparse
import logging
import sys
import os
from typing import List, Tuple
import numpy as np

# Handle matplotlib for headless environments
import matplotlib
if os.environ.get('DISPLAY') is None:
    matplotlib.use('Agg')  # Non-interactive backend
    HEADLESS = True
else:
    HEADLESS = False

import matplotlib.pyplot as plt

# Add parent directory to path if needed
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from jidenna import RobotAPI, RobotState
from navigation import LocalPlanner, PlannerConfig, PlannerState

class PathGenerator:
    """Path generator with smooth curves"""
    
    @staticmethod
    def straight_line(start=(0, 0), end=(3, 0), num_points=30):
        """Generate straight line path"""
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            points.append((x, y))
        return points
    
    @staticmethod
    def smooth_turn(start=(0, 0), turn_point=(0, 1), end=(1, 1), 
                   turn_radius=0.3, num_points=40):
        """
        Generate smooth 90-degree turn using arc
        
        Args:
            start: Starting point
            turn_point: Point where turn begins
            end: End point
            turn_radius: Radius of the turning arc
            num_points: Total number of path points
        """
        points = []
        
        # Calculate distances
        dist_to_turn = math.sqrt((turn_point[0] - start[0])**2 + 
                                 (turn_point[1] - start[1])**2)
        dist_after_turn = math.sqrt((end[0] - turn_point[0])**2 + 
                                    (end[1] - turn_point[1])**2)
        
        # Reduce distances to account for turn radius
        straight1_dist = max(0, dist_to_turn - turn_radius)
        straight2_dist = max(0, dist_after_turn - turn_radius)
        
        # Number of points for each section
        n_straight1 = int(num_points * 0.3)
        n_turn = int(num_points * 0.4)
        n_straight2 = num_points - n_straight1 - n_turn
        
        # First straight section
        for i in range(n_straight1):
            t = i / max(1, n_straight1 - 1)
            x = start[0] + t * (turn_point[0] - start[0]) * (straight1_dist / dist_to_turn)
            y = start[1] + t * (turn_point[1] - start[1]) * (straight1_dist / dist_to_turn)
            points.append((x, y))
        
        # Turning arc (quarter circle)
        # Determine direction of turn
        dx1 = turn_point[0] - start[0]
        dy1 = turn_point[1] - start[1]
        dx2 = end[0] - turn_point[0]
        dy2 = end[1] - turn_point[1]
        
        # Cross product to determine turn direction
        cross = dx1 * dy2 - dy1 * dx2
        turn_direction = 1 if cross > 0 else -1
        
        # Arc center
        if abs(dx1) > abs(dy1):
            # Moving mostly in X direction
            if dx1 > 0:
                center_x = turn_point[0] - turn_radius
            else:
                center_x = turn_point[0] + turn_radius
            center_y = turn_point[1]
            start_angle = math.pi if dx1 > 0 else 0
        else:
            # Moving mostly in Y direction
            center_x = turn_point[0]
            if dy1 > 0:
                center_y = turn_point[1] - turn_radius
            else:
                center_y = turn_point[1] + turn_radius
            start_angle = math.pi/2 if dy1 > 0 else -math.pi/2
        
        # Generate arc points
        for i in range(n_turn):
            t = i / max(1, n_turn - 1)
            angle = start_angle + turn_direction * (math.pi/2) * t
            x = center_x + turn_radius * math.cos(angle)
            y = center_y + turn_radius * math.sin(angle)
            points.append((x, y))
        
        # Second straight section
        for i in range(n_straight2):
            t = i / max(1, n_straight2 - 1)
            x = turn_point[0] + t * (end[0] - turn_point[0]) * (straight2_dist / dist_after_turn)
            y = turn_point[1] + t * (end[1] - turn_point[1]) * (straight2_dist / dist_after_turn)
            points.append((x, y))
        
        return points
    
    @staticmethod
    def right_angle_path(start=(0, 0), intermediate=(0, 2), end=(3, 2), 
                        turn_radius=0.3, num_points=40):
        """Generate 90-degree path with smooth turn"""
        return PathGenerator.smooth_turn(start, intermediate, end, turn_radius, num_points)
    
    @staticmethod
    def s_curve(start=(0, 0), end=(3, 0), amplitude=0.5, num_points=40):
        """Generate S-curve path"""
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1]) + amplitude * math.sin(math.pi * t)
            points.append((x, y))
        return points
    
    @staticmethod
    def circle(center=(0, 0), radius=1.0, num_points=40):
        """Generate circular path"""
        points = []
        for i in range(num_points):
            angle = (2 * math.pi * i) / (num_points - 1)
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            points.append((x, y))
        return points

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('robot_test.log')
    ]
)
logger = logging.getLogger(__name__)

class RealRobotTester:
    """Safe tester for real robot with IMU integration"""
    
    def __init__(self, port: str = None, use_imu: bool = True):
        """Initialize tester"""
        self.robot = RobotAPI(port=port)
        self.use_imu = use_imu
        self.headless = HEADLESS
        
        # Conservative planner config for real robot with IMU
        config = PlannerConfig(
            lookahead_distance=0.5,  # Increased for smoother turns
            lookahead_min=0.3,
            lookahead_max=1.0,      # Increased for turns
            max_linear_velocity=0.2,  # Start VERY slow
            max_angular_velocity=0.5,  # Limited rotation
            position_tolerance=0.15,
            heading_tolerance=math.radians(30),
            goal_slowdown_distance=0.4,
            # IMU settings
            use_imu_heading=use_imu,
            imu_heading_weight=0.7,
            max_heading_discrepancy=math.radians(30),
            # Curvature settings
            max_curvature=3.0,  # Increased for sharper turns
            final_approach_distance=0.3,
            final_approach_speed=0.1,
            min_approach_speed=0.03
        )
        self.planner = LocalPlanner(config)
        
        # Safety flags
        self.emergency_stop = False
        self.connection_lost = False
        self.last_telemetry_time = 0
        self.telemetry_timeout = 1.0
        
        # Data logging
        self.data_log = []
        self.start_time = 0
    
    def connect(self) -> bool:
        """Connect to robot safely"""
        logger.info(f"Connecting to robot...")
        if self.robot.connect():
            logger.info("Connected successfully!")
            time.sleep(1.0)
            state = self.robot.get_state()
            if state.is_valid():
                logger.info(f"Telemetry OK - Position: ({state.x:.2f}, {state.y:.2f}), "
                           f"Heading: {math.degrees(state.heading):.1f}°")
                
                if self.use_imu and state.is_imu_valid():
                    logger.info(f"IMU OK - Angle Z: {state.imu_angle_z:.1f}°, "
                               f"Gyro Z: {state.imu_gyro_z:.1f}°/s")
                    logger.info(f"Heading discrepancy: {math.degrees(state.heading_discrepancy):.1f}°")
                elif self.use_imu:
                    logger.warning("IMU data invalid - using odometry only")
                    self.use_imu = False
                    self.planner.config.use_imu_heading = False
                
                return True
            else:
                logger.error("Telemetry invalid - check ESP32 connection")
                self.robot.disconnect()
                return False
        else:
            logger.error("Failed to connect - check port and power")
            return False
    
    def check_safety(self, state: RobotState) -> bool:
        """Check all safety conditions"""
        if not state.is_valid():
            logger.error("SAFETY: Invalid robot state")
            return False
        
        if any([math.isnan(v) for v in [state.x, state.y, state.heading]]):
            logger.error("SAFETY: NaN values in state")
            return False
        
        current_time = time.time()
        if state.timestamp > 0:
            self.last_telemetry_time = current_time
        elif current_time - self.last_telemetry_time > self.telemetry_timeout:
            logger.error("SAFETY: Telemetry timeout")
            return False
        
        max_allowed_v = 0.5
        max_allowed_w = 1.0
        
        if abs(state.v) > max_allowed_v:
            logger.error(f"SAFETY: Linear velocity too high: {state.v:.2f} m/s")
            return False
        
        if abs(state.w) > max_allowed_w:
            logger.error(f"SAFETY: Angular velocity too high: {state.w:.2f} rad/s")
            return False
        
        if self.use_imu and not state.is_imu_valid():
            logger.warning("SAFETY: IMU data invalid")
            self.use_imu = False
            self.planner.config.use_imu_heading = False
        
        return True
    
    def emergency_stop(self):
        """Emergency stop the robot"""
        logger.warning("EMERGENCY STOP!")
        self.emergency_stop = True
        self.robot.stop()
        time.sleep(0.1)
        self.robot.stop()
    
    def run_safe_test(self, path_points: List[Tuple[float, float]], 
                      test_name: str = "real_test",
                      max_duration: float = 30.0):
        """Run test with safety monitoring and IMU fusion"""
        logger.info(f"Starting real robot test: {test_name}")
        logger.info("SAFETY: Keep remote/switch ready for manual stop!")
        logger.info(f"IMU Fusion: {'ENABLED' if self.use_imu else 'DISABLED'}")
        
        self.planner.set_path(path_points)
        self.start_time = time.time()
        
        positions = []
        velocities = []
        distances = []
        timestamps = []
        imu_angles = []
        heading_discrepancies = []
        
        if not self.headless:
            plt.ion()
            fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))
        
        path_x = [p[0] for p in path_points]
        path_y = [p[1] for p in path_points]
        
        try:
            while time.time() - self.start_time < max_duration:
                state = self.robot.get_state()
                
                if not self.check_safety(state):
                    self.emergency_stop()
                    logger.error("Test aborted due to safety check failure")
                    break
                
                if self.emergency_stop:
                    logger.warning("Test stopped manually")
                    break
                
                robot_pose = (state.x, state.y, state.heading)
                
                # Update planner with IMU data
                if self.use_imu and state.is_imu_valid():
                    v, w = self.planner.update(
                        robot_pose, 
                        imu_heading=state.imu_heading
                    )
                else:
                    v, w = self.planner.update(robot_pose)
                
                # Safety limits
                if abs(v) > 0.25:
                    v = 0.25 * (1 if v > 0 else -1)
                
                if abs(w) > 0.6:
                    w = 0.6 * (1 if w > 0 else -1)
                
                self.robot.set_velocity(v, w)
                
                positions.append(robot_pose)
                velocities.append((v, w))
                distances.append(self.planner.distance_to_goal)
                timestamps.append(time.time() - self.start_time)
                imu_angles.append(state.imu_angle_z if state.is_imu_valid() else 0)
                heading_discrepancies.append(
                    math.degrees(self.planner.heading_discrepancy) 
                    if hasattr(self.planner, 'heading_discrepancy') else 0
                )
                
                if not self.headless and len(positions) % 5 == 0:
                    self._update_visualization(ax1, ax2, ax3, path_points, positions, 
                                              velocities, distances, timestamps,
                                              imu_angles, heading_discrepancies)
                
                if time.time() - self.start_time > 2 and len(positions) % 20 == 0:
                    logger.info(f"Progress: dist={self.planner.distance_to_goal:.2f}m, "
                               f"v={v:.2f}, w={w:.2f}, "
                               f"pos=({state.x:.2f}, {state.y:.2f}), "
                               f"heading={math.degrees(state.heading):.1f}°")
                
                if self.planner.state == PlannerState.GOAL_REACHED:
                    logger.info("GOAL REACHED!")
                    break
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            logger.info("Test interrupted by user")
            self.emergency_stop()
        except Exception as e:
            logger.error(f"Test error: {e}")
            import traceback
            traceback.print_exc()
            self.emergency_stop()
        finally:
            self.robot.stop()
            time.sleep(0.1)
            self.robot.stop()
            logger.info("Robot stopped")
            
            if not self.headless:
                plt.ioff()
                plt.show()
                plt.savefig(f'{test_name}_results.png')
                logger.info(f"Results saved to {test_name}_results.png")
        
        return {
            'test_name': test_name,
            'success': self.planner.state == PlannerState.GOAL_REACHED,
            'final_state': self.planner.state,
            'duration': time.time() - self.start_time,
            'final_position': positions[-1] if positions else None,
            'final_distance': distances[-1] if distances else None,
            'max_velocity': max([abs(v[0]) for v in velocities]) if velocities else 0,
            'avg_heading_discrepancy': np.mean(heading_discrepancies) if heading_discrepancies else 0,
            'max_heading_discrepancy': max(heading_discrepancies) if heading_discrepancies else 0
        }
    
    def _update_visualization(self, ax1, ax2, ax3, path_points, positions, 
                             velocities, distances, timestamps,
                             imu_angles, heading_discrepancies):
        """Update visualization plots"""
        ax1.clear()
        ax2.clear()
        ax3.clear()
        
        path_x = [p[0] for p in path_points]
        path_y = [p[1] for p in path_points]
        ax1.plot(path_x, path_y, 'b-', label='Planned Path', linewidth=2)
        
        if positions:
            pos_x = [p[0] for p in positions]
            pos_y = [p[1] for p in positions]
            ax1.plot(pos_x, pos_y, 'r-', label='Actual Path', linewidth=1.5)
            
            current = positions[-1]
            ax1.plot(current[0], current[1], 'go', markersize=8)
            
            arrow_len = 0.2
            dx = arrow_len * math.cos(current[2])
            dy = arrow_len * math.sin(current[2])
            ax1.arrow(current[0], current[1], dx, dy, 
                     head_width=0.08, head_length=0.12, fc='g', ec='g')
        
        ax1.legend()
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Robot Path Tracking')
        ax1.grid(True)
        ax1.axis('equal')
        
        if timestamps:
            ax2.plot(timestamps, distances, 'b-', label='Distance to Goal')
            ax2.plot(timestamps, [abs(v[0]) for v in velocities], 'r-', label='Linear Velocity')
            ax2.plot(timestamps, [abs(v[1]) for v in velocities], 'g-', label='Angular Velocity')
            ax2.legend()
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Value')
            ax2.set_title('Progress Metrics')
            ax2.grid(True)
        
        if timestamps:
            ax3.plot(timestamps, imu_angles, 'b-', label='IMU Angle')
            ax3.plot(timestamps, heading_discrepancies, 'r-', label='Heading Discrepancy')
            ax3.legend()
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Angle (degrees)')
            ax3.set_title('IMU Data')
            ax3.grid(True)
        
        plt.pause(0.01)
    
    def disconnect(self):
        """Disconnect from robot safely"""
        self.robot.stop()
        time.sleep(0.1)
        self.robot.disconnect()
        logger.info("Disconnected from robot")


def main():
    """Main function for real robot testing"""
    parser = argparse.ArgumentParser(description='Test real robot navigation')
    parser.add_argument('--port', type=str, help='Serial port (e.g., COM3, /dev/ttyUSB0)')
    parser.add_argument('--test', type=str, 
                       choices=['straight', 'short', 'turn', 'smooth_turn', 'square'],
                       default='short',
                       help='Test to run')
    parser.add_argument('--no-imu', action='store_true', 
                       help='Disable IMU fusion')
    parser.add_argument('--list-ports', action='store_true', 
                       help='List available serial ports')
    parser.add_argument('--no-viz', action='store_true',
                       help='Disable visualization')
    
    args = parser.parse_args()
    
    global HEADLESS
    if args.no_viz:
        HEADLESS = True
    
    if args.list_ports:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        print("\nAvailable Serial Ports:")
        for port in ports:
            print(f"  {port.device}: {port.description}")
        return
    
    tester = RealRobotTester(port=args.port, use_imu=not args.no_imu)
    
    try:
        if not tester.connect():
            return
        
        # Define test paths
        if args.test == 'straight':
            path = PathGenerator.straight_line(start=(0, 0), end=(2, 0), num_points=20)
        elif args.test == 'short':
            path = PathGenerator.straight_line(start=(0, 0), end=(1, 0), num_points=10)
        elif args.test == 'turn':
            # Old sharp turn (for comparison)
            path = PathGenerator.right_angle_path(
                start=(0, 0), intermediate=(0, 1), end=(1, 1), 
                turn_radius=0.1, num_points=20
            )
        elif args.test == 'smooth_turn':
            # New smooth turn
            path = PathGenerator.smooth_turn(
                start=(0, 0), turn_point=(0, 1), end=(1, 1), 
                turn_radius=0.3, num_points=30
            )
        elif args.test == 'square':
            # 1m square with smooth corners
            path = [
                (0, 0), (0.25, 0), (0.5, 0), (0.75, 0), (1, 0),
                (1, 0.25), (1, 0.5), (1, 0.75), (1, 1),
                (0.75, 1), (0.5, 1), (0.25, 1), (0, 1),
                (0, 0.75), (0, 0.5), (0, 0.25), (0, 0)
            ]
        
        logger.info(f"Test path: {args.test}")
        logger.info(f"Path length: {len(path)} points")
        logger.info(f"IMU Fusion: {'DISABLED' if args.no_imu else 'ENABLED'}")
        logger.info("SAFETY REMINDERS:")
        logger.info("  1. Ensure robot has clear space")
        logger.info("  2. Keep emergency stop ready")
        logger.info("  3. Start with short distances")
        logger.info("  4. Monitor battery level")
        
        input("Press ENTER to start test...")
        
        results = tester.run_safe_test(path, f"real_{args.test}", max_duration=20.0)
        
        logger.info("\n" + "="*50)
        logger.info("TEST RESULTS")
        logger.info("="*50)
        logger.info(f"Test: {results['test_name']}")
        logger.info(f"Success: {results['success']}")
        logger.info(f"Final State: {results['final_state']}")
        logger.info(f"Duration: {results['duration']:.2f}s")
        logger.info(f"Final Position: {results['final_position']}")
        logger.info(f"Final Distance: {results['final_distance']:.3f}m")
        logger.info(f"Max Velocity: {results['max_velocity']:.3f} m/s")
        logger.info(f"Avg Heading Discrepancy: {results['avg_heading_discrepancy']:.2f}°")
        logger.info(f"Max Heading Discrepancy: {results['max_heading_discrepancy']:.2f}°")
        
    except KeyboardInterrupt:
        logger.info("Program interrupted")
    except Exception as e:
        logger.error(f"Program error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.disconnect()


if __name__ == "__main__":
    main()