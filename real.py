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
    """Path generator optimized for differential drive robot"""
    
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
    def l_shape_path(size=1.0, num_points=20):
        """
        Generate L-shaped path (2 turns instead of 4)
        Easier than square for odometry-based navigation
        
        Path: (0,0) → (size,0) → (size,size)
        """
        points = []
        
        # First segment: go right along X axis
        n_first = num_points // 2
        for i in range(n_first):
            t = i / max(1, n_first - 1)
            x = t * size
            y = 0
            points.append((x, y))
        
        # Second segment: go up along Y axis
        n_second = num_points - n_first
        for i in range(n_second):
            t = i / max(1, n_second - 1)
            x = size
            y = t * size
            points.append((x, y))
        
        return points
    
    @staticmethod
    def smooth_90_turn(start=(0, 0), turn_center=(0.5, 0.5), end=(1, 1), 
                       turn_radius=0.5, num_points=50):
        """
        Generate smooth 90-degree turn with proper radius
        
        For a robot with 0.521m wheel separation, minimum turn radius is ~0.26m
        Recommended turn radius: 0.4-0.6m for smooth operation
        """
        points = []
        
        # Calculate distances from turn center to start/end
        dx_start = turn_center[0] - start[0]
        dy_start = turn_center[1] - start[1]
        dist_start = max(1e-6, math.sqrt(dx_start**2 + dy_start**2))
        
        dx_end = end[0] - turn_center[0]
        dy_end = end[1] - turn_center[1]
        dist_end = max(1e-6, math.sqrt(dx_end**2 + dy_end**2))
        
        # Number of points for each section
        n_approach = int(num_points * 0.3)
        n_turn = int(num_points * 0.5)
        n_depart = num_points - n_approach - n_turn
        
        # Approach section (straight line to start of turn arc)
        approach_end = (
            turn_center[0] + dx_start * turn_radius / dist_start,
            turn_center[1] + dy_start * turn_radius / dist_start
        )
        
        for i in range(n_approach):
            t = i / max(1, n_approach - 1)
            x = start[0] + t * (approach_end[0] - start[0])
            y = start[1] + t * (approach_end[1] - start[1])
            points.append((x, y))
        
        # Turn section (arc with specified radius)
        angle_start = math.atan2(dy_start, dx_start)
        angle_end = math.atan2(dy_end, dx_end)
        
        angle_diff = angle_end - angle_start
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        for i in range(n_turn):
            t = i / max(1, n_turn - 1)
            angle = angle_start + angle_diff * t
            x = turn_center[0] + turn_radius * math.cos(angle)
            y = turn_center[1] + turn_radius * math.sin(angle)
            points.append((x, y))
        
        # Depart section (straight line from end of turn arc)
        depart_start = (
            turn_center[0] + dx_end * turn_radius / dist_end,
            turn_center[1] + dy_end * turn_radius / dist_end
        )
        
        for i in range(n_depart):
            t = i / max(1, n_depart - 1)
            x = depart_start[0] + t * (end[0] - depart_start[0])
            y = depart_start[1] + t * (end[1] - depart_start[1])
            points.append((x, y))
        
        return points
    
    @staticmethod
    def square_path(size=1.0, turn_radius=0.4, num_points_per_side=20):
        """
        Generate square path with smooth corners
        """
        points = []
        
        half = size / 2
        corners = [
            (-half, -half),  # Bottom-left
            (half, -half),   # Bottom-right
            (half, half),    # Top-right
            (-half, half)    # Top-left
        ]
        
        for i in range(4):
            current = corners[i]
            next_corner = corners[(i + 1) % 4]
            prev_corner = corners[(i - 1) % 4]
            
            dx_in = (current[0] - prev_corner[0])
            dy_in = (current[1] - prev_corner[1])
            dist_in = max(1e-6, math.sqrt(dx_in**2 + dy_in**2))
            dx_in /= dist_in
            dy_in /= dist_in
            
            dx_out = (next_corner[0] - current[0])
            dy_out = (next_corner[1] - current[1])
            dist_out = max(1e-6, math.sqrt(dx_out**2 + dy_out**2))
            dx_out /= dist_out
            dy_out /= dist_out
            
            turn_center_x = current[0] + (dx_in - dx_out) * turn_radius
            turn_center_y = current[1] + (dy_in - dy_out) * turn_radius
            
            arc_start_x = current[0] - dx_in * turn_radius
            arc_start_y = current[1] - dy_in * turn_radius
            arc_end_x = current[0] + dx_out * turn_radius
            arc_end_y = current[1] + dy_out * turn_radius
            
            straight_start = (
                prev_corner[0] + dx_in * turn_radius,
                prev_corner[1] + dy_in * turn_radius
            )
            
            n_straight = num_points_per_side // 2
            for j in range(n_straight):
                t = j / max(1, n_straight - 1)
                x = straight_start[0] + t * (arc_start_x - straight_start[0])
                y = straight_start[1] + t * (arc_start_y - straight_start[1])
                points.append((x, y))
            
            n_arc = num_points_per_side // 2
            angle_start = math.atan2(arc_start_y - turn_center_y, arc_start_x - turn_center_x)
            angle_end = math.atan2(arc_end_y - turn_center_y, arc_end_x - turn_center_x)
            
            angle_diff = angle_end - angle_start
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            for j in range(n_arc):
                t = j / max(1, n_arc - 1)
                angle = angle_start + angle_diff * t
                x = turn_center_x + turn_radius * math.cos(angle)
                y = turn_center_y + turn_radius * math.sin(angle)
                points.append((x, y))
        
        points.append(points[0])
        
        return points
    
    @staticmethod
    def circle(center=(0, 0), radius=1.0, num_points=50):
        """Generate circular path"""
        points = []
        for i in range(num_points):
            angle = (2 * math.pi * i) / (num_points - 1)
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            points.append((x, y))
        return points
    
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
            lookahead_distance=0.5,
            lookahead_min=0.3,
            lookahead_max=1.0,
            max_linear_velocity=0.2,
            max_angular_velocity=0.5,
            position_tolerance=0.15,
            heading_tolerance=math.radians(30),
            goal_slowdown_distance=0.4,
            # IMU settings
            use_imu_heading=use_imu,
            imu_heading_weight=0.7,
            max_heading_discrepancy=math.radians(30),
            # Curvature settings
            max_curvature=3.0,
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
                       choices=['straight', 'short', 'wide_turn', 'l_shape', 'square', 'circle', 's_curve'],
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
        
        # Define test paths with proper turn radii
        if args.test == 'straight':
            path = PathGenerator.straight_line(start=(0, 0), end=(2, 0), num_points=10)
            print(path)
        elif args.test == 'short':
            path = PathGenerator.straight_line(start=(0, 0), end=(1, 0), num_points=10)
        elif args.test == 'wide_turn':
            # Wide turn with 0.5m radius (recommended for this robot)
            path = PathGenerator.smooth_90_turn(
                start=(0, 0), 
                turn_center=(0.5, 0.5), 
                end=(1, 1), 
                turn_radius=0.5, 
                num_points=50
            )
        elif args.test == 'l_shape':
            # L-shaped path (2 turns instead of 4)
            path = PathGenerator.l_shape_path(size=1.0, num_points=20)
        elif args.test == 'square':
            # 1m square with 0.4m turn radius
            path = PathGenerator.square_path(size=1.0, turn_radius=0.4)
        elif args.test == 'circle':
            # Circle with 1m radius
            path = PathGenerator.circle(radius=1.0, num_points=50)
        elif args.test == 's_curve':
            # S-curve
            path = PathGenerator.s_curve(start=(0, 0), end=(2, 0), amplitude=0.3, num_points=40)
        
        logger.info(f"Test path: {args.test}")
        logger.info(f"Path length: {len(path)} points")
        logger.info(f"IMU Fusion: {'DISABLED' if args.no_imu else 'ENABLED'}")
        logger.info("SAFETY REMINDERS:")
        logger.info("  1. Ensure robot has clear space")
        logger.info("  2. Keep emergency stop ready")
        logger.info("  3. Start with short distances")
        logger.info("  4. Monitor battery level")
        
        input("Press ENTER to start test...")
        
        results = tester.run_safe_test(path, f"real_{args.test}", max_duration=30.0)
        
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