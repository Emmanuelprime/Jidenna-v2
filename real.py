#!/usr/bin/env python3
"""
Safe test program for real robot
Tests local planner with real ESP32 hardware
"""

import math
import time
import argparse
import logging
import sys
import os
from typing import List, Tuple
import numpy as np
import matplotlib.pyplot as plt

# Add parent directory to path if needed
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from jidenna import RobotAPI, RobotState
from navigation import LocalPlanner, PlannerConfig, PlannerState
from main import PathGenerator

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
    """Safe tester for real robot"""
    
    def __init__(self, port: str = None):
        """Initialize tester"""
        self.robot = RobotAPI(port=port)
        
        # Conservative planner config for real robot
        config = PlannerConfig(
            lookahead_distance=0.3,
            lookahead_min=0.2,
            lookahead_max=0.6,
            max_linear_velocity=0.2,  # Start VERY slow
            max_angular_velocity=0.5,  # Limited rotation
            position_tolerance=0.15,
            heading_tolerance=math.radians(30),
            goal_slowdown_distance=0.4
        )
        self.planner = LocalPlanner(config)
        
        # Safety flags
        self.emergency_stop = False
        self.connection_lost = False
        self.last_telemetry_time = 0
        self.telemetry_timeout = 1.0  # 1 second timeout
        
        # Data logging
        self.data_log = []
        self.start_time = 0
    
    def connect(self) -> bool:
        """Connect to robot safely"""
        logger.info(f"Connecting to robot...")
        if self.robot.connect():
            logger.info("Connected successfully!")
            # Verify telemetry is working
            time.sleep(1.0)
            state = self.robot.get_state()
            if state.is_valid():
                logger.info(f"Telemetry OK - Position: ({state.x:.2f}, {state.y:.2f}), "
                           f"Heading: {math.degrees(state.heading):.1f}°")
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
        # Check if state is valid
        if not state.is_valid():
            logger.error("SAFETY: Invalid robot state")
            return False
        
        # Check for NaN or Inf values
        if any([math.isnan(v) for v in [state.x, state.y, state.heading]]):
            logger.error("SAFETY: NaN values in state")
            return False
        
        # Check telemetry timeout
        current_time = time.time()
        if state.timestamp > 0:
            self.last_telemetry_time = current_time
        elif current_time - self.last_telemetry_time > self.telemetry_timeout:
            logger.error("SAFETY: Telemetry timeout")
            return False
        
        # Check if velocities are within limits
        max_allowed_v = 0.5  # Hard safety limit
        max_allowed_w = 1.0  # Hard safety limit
        
        if abs(state.v) > max_allowed_v:
            logger.error(f"SAFETY: Linear velocity too high: {state.v:.2f} m/s")
            return False
        
        if abs(state.w) > max_allowed_w:
            logger.error(f"SAFETY: Angular velocity too high: {state.w:.2f} rad/s")
            return False
        
        return True
    
    def emergency_stop(self):
        """Emergency stop the robot"""
        logger.warning("EMERGENCY STOP!")
        self.emergency_stop = True
        self.robot.stop()
        time.sleep(0.1)
        self.robot.stop()  # Double stop for safety
    
    def run_safe_test(self, path_points: List[Tuple[float, float]], 
                      test_name: str = "real_test",
                      max_duration: float = 30.0):
        """Run test with safety monitoring"""
        logger.info(f"Starting real robot test: {test_name}")
        logger.info("SAFETY: Keep remote/switch ready for manual stop!")
        
        # Set path
        self.planner.set_path(path_points)
        self.start_time = time.time()
        
        # Initialize data storage
        positions = []
        velocities = []
        distances = []
        timestamps = []
        
        # Setup visualization
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        path_x = [p[0] for p in path_points]
        path_y = [p[1] for p in path_points]
        
        try:
            while time.time() - self.start_time < max_duration:
                # Get current state
                state = self.robot.get_state()
                
                # Check safety
                if not self.check_safety(state):
                    self.emergency_stop()
                    logger.error("Test aborted due to safety check failure")
                    break
                
                # Check for manual stop (keyboard)
                if self.emergency_stop:
                    logger.warning("Test stopped manually")
                    break
                
                robot_pose = (state.x, state.y, state.heading)
                
                # Update planner
                v, w = self.planner.update(robot_pose)
                
                # Additional safety checks on velocity commands
                if abs(v) > 0.2:  # Hard limit for first tests
                    logger.warning(f"Limiting velocity: {v:.3f} -> 0.2")
                    v = 0.2 * (1 if v > 0 else -1)
                
                if abs(w) > 0.5:  # Hard limit for first tests
                    logger.warning(f"Limiting angular velocity: {w:.3f} -> 0.5")
                    w = 0.5 * (1 if w > 0 else -1)
                
                # Send velocity command
                self.robot.set_velocity(v, w)
                
                # Store data
                positions.append(robot_pose)
                velocities.append((v, w))
                distances.append(self.planner.distance_to_goal)
                timestamps.append(time.time() - self.start_time)
                
                # Update visualization
                if len(positions) % 5 == 0:
                    self._update_visualization(ax1, ax2, path_points, positions, 
                                              velocities, distances, timestamps)
                
                # Log progress every 2 seconds
                if time.time() - self.start_time > 2 and len(positions) % 40 == 0:
                    logger.info(f"Progress: dist={self.planner.distance_to_goal:.2f}m, "
                               f"v={v:.2f}, w={w:.2f}, "
                               f"pos=({state.x:.2f}, {state.y:.2f})")
                
                # Check if goal reached
                if self.planner.state == PlannerState.GOAL_REACHED:
                    logger.info("GOAL REACHED!")
                    break
                
                # Control loop rate
                time.sleep(0.05)  # 20 Hz
                
        except KeyboardInterrupt:
            logger.info("Test interrupted by user")
            self.emergency_stop()
        except Exception as e:
            logger.error(f"Test error: {e}")
            import traceback
            traceback.print_exc()
            self.emergency_stop()
        finally:
            # Always stop the robot
            self.robot.stop()
            time.sleep(0.1)
            self.robot.stop()
            logger.info("Robot stopped")
            
            # Final visualization
            plt.ioff()
            plt.show()
        
        # Return results
        return {
            'test_name': test_name,
            'success': self.planner.state == PlannerState.GOAL_REACHED,
            'final_state': self.planner.state,
            'duration': time.time() - self.start_time,
            'final_position': positions[-1] if positions else None,
            'final_distance': distances[-1] if distances else None,
            'max_velocity': max([abs(v[0]) for v in velocities]) if velocities else 0
        }
    
    def _update_visualization(self, ax1, ax2, path_points, positions, 
                             velocities, distances, timestamps):
        """Update visualization plots"""
        # Clear and redraw
        ax1.clear()
        ax2.clear()
        
        # Path plot
        path_x = [p[0] for p in path_points]
        path_y = [p[1] for p in path_points]
        ax1.plot(path_x, path_y, 'b-', label='Planned Path', linewidth=2)
        
        if positions:
            pos_x = [p[0] for p in positions]
            pos_y = [p[1] for p in positions]
            ax1.plot(pos_x, pos_y, 'r-', label='Actual Path', linewidth=1.5)
            
            # Current position
            current = positions[-1]
            ax1.plot(current[0], current[1], 'go', markersize=8)
            
            # Heading
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
        
        # Velocity and distance plot
        if timestamps:
            ax2.plot(timestamps, distances, 'b-', label='Distance to Goal')
            ax2.plot(timestamps, [abs(v[0]) for v in velocities], 'r-', label='Linear Velocity')
            ax2.plot(timestamps, [abs(v[1]) for v in velocities], 'g-', label='Angular Velocity')
            ax2.legend()
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Value')
            ax2.set_title('Progress Metrics')
            ax2.grid(True)
        
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
                       choices=['straight', 'short', 'turn', 'square'],
                       default='short',
                       help='Test to run')
    parser.add_argument('--list-ports', action='store_true', 
                       help='List available serial ports')
    
    args = parser.parse_args()
    
    # List ports if requested
    if args.list_ports:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        print("\nAvailable Serial Ports:")
        for port in ports:
            print(f"  {port.device}: {port.description}")
        return
    
    # Create tester
    tester = RealRobotTester(port=args.port)
    
    try:
        # Connect
        if not tester.connect():
            return
        
        # Define test paths (SHORT distances for safety)
        if args.test == 'straight':
            # 2m straight line
            path = PathGenerator.straight_line(start=(0, 0), end=(2, 0), num_points=20)
        elif args.test == 'short':
            # 1m straight line (safest first test)
            path = PathGenerator.straight_line(start=(0, 0), end=(1, 0), num_points=10)
        elif args.test == 'turn':
            # 1m forward, 90° turn, 1m forward
            path = PathGenerator.right_angle_path(
                start=(0, 0), intermediate=(0, 1), end=(1, 1), num_points=20
            )
        elif args.test == 'square':
            # 1m square
            path = [
                (0, 0), (0.5, 0), (1, 0),
                (1, 0.5), (1, 1),
                (0.5, 1), (0, 1),
                (0, 0.5), (0, 0)
            ]
        
        logger.info(f"Test path: {args.test}")
        logger.info(f"Path length: {len(path)} points")
        logger.info("SAFETY REMINDERS:")
        logger.info("  1. Ensure robot has clear space")
        logger.info("  2. Keep emergency stop ready")
        logger.info("  3. Start with short distances")
        logger.info("  4. Monitor battery level")
        
        input("Press ENTER to start test...")
        
        # Run test
        results = tester.run_safe_test(path, f"real_{args.test}", max_duration=20.0)
        
        # Print results
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