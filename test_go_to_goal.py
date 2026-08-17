#!/usr/bin/env python3
"""
Simple test script for GoToGoal Controller

Usage:
    python test_go_to_goal.py --goal 2.0 2.0 --port /dev/ttyUSB0
    python test_go_to_goal.py --goal 0.0 0.0 --port /dev/ttyUSB0
    python test_go_to_goal.py --goal 1.5 1.5 --port COM3 --speed 0.3
"""

import sys
import time
import argparse
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

try:
    from jidenna.robot import RobotAPI
    from navigation.g2g import GoToGoal, GoToGoalConfig
except ImportError as e:
    print(f"Error: {e}")
    print("Make sure go_to_goal.py is in the navigation directory")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="GoToGoal Test")
    parser.add_argument('--goal', type=float, nargs=2, required=True,
                       help='Goal position (x y) in meters')
    parser.add_argument('--port', type=str, required=True,
                       help='Serial port for robot (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--speed', type=float, default=0.25,
                       help='Maximum linear velocity (m/s)')
    parser.add_argument('--timeout', type=float, default=60.0,
                       help='Navigation timeout (seconds)')
    parser.add_argument('--no-imu', action='store_true',
                       help='Disable IMU fusion')
    parser.add_argument('--list-ports', action='store_true',
                       help='List available serial ports')
    
    args = parser.parse_args()
    
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
    
    # Connect to robot
    robot = RobotAPI(args.port)
    if not robot.connect():
        print(f"Failed to connect to {args.port}")
        return 1
    
    logger.info(f"Connected to robot on {args.port}")
    
    # Create config
    config = GoToGoalConfig()
    config.max_linear_speed = args.speed
    config.max_time = args.timeout
    config.use_imu = not args.no_imu
    
    # Create controller
    gtg = GoToGoal(robot, config)
    
    try:
        # Get starting position
        state = robot.get_state()
        logger.info(f"Starting at: ({state.x:.2f}, {state.y:.2f})")
        
        # Define callback for status updates
        def status_callback(status):
            if int(status['elapsed_time']) > int(status['elapsed_time'] - 1):
                print(f"  dist={status['distance_to_goal']:.3f}m, "
                      f"heading_err={status['heading_error_deg']:.1f}deg, "
                      f"state={status['state']}")
        
        # Run navigation
        goal_x, goal_y = args.goal
        logger.info(f"Going to goal: ({goal_x:.2f}, {goal_y:.2f})")
        print("Press Ctrl+C to stop")
        
        success = gtg.go_to(goal_x, goal_y, callback=status_callback)
        
        # Results
        status = gtg.get_status()
        print("\n" + "="*50)
        print("RESULTS")
        print("="*50)
        print(f"Goal: ({goal_x:.2f}, {goal_y:.2f})")
        print(f"Final Position: ({status['x']:.2f}, {status['y']:.2f})")
        print(f"Final Distance: {status['distance_to_goal']:.3f}m")
        print(f"Heading Error: {status['heading_error_deg']:.1f}deg")
        print(f"Time: {status['elapsed_time']:.1f}s")
        print(f"Goal Reached: {'YES' if success else 'NO'}")
        print("="*50)
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        return 1
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        robot.stop()
        robot.disconnect()
        logger.info("Robot disconnected")


if __name__ == "__main__":
    sys.exit(main())