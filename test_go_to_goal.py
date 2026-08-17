#!/usr/bin/env python3
"""
Test script for GoToGoal Controller with Odometry Reset

This script tests the go-to-goal controller with odometry reset support.
It goes to a goal and then returns to origin with odometry reset.

Usage:
    python test_g2g_reset.py --port /dev/ttyUSB0 --goal 1.0 1.0
    python test_g2g_reset.py --port /dev/ttyUSB0 --goals 1.0,1.0 0.0,0.0
"""

import sys
import time
import math
import argparse
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

try:
    from jidenna.robot import RobotAPI
    from navigation.g2g import GoToGoal, GoToGoalConfig, GoToGoalState
except ImportError as e:
    print(f"Error: {e}")
    sys.exit(1)


def status_callback(status):
    """Simple status callback"""
    if int(status['elapsed_time']) > int(status['elapsed_time'] - 1):
        print(f"  dist={status['distance_to_goal']:.3f}m, "
              f"heading_err={status['heading_error_deg']:.1f}deg, "
              f"state={status['state']}, "
              f"pos=({status['x']:.2f}, {status['y']:.2f})")


def main():
    parser = argparse.ArgumentParser(description="GoToGoal with Odometry Reset")
    parser.add_argument('--port', type=str, required=True,
                       help='Serial port for robot')
    parser.add_argument('--goal', type=float, nargs=2, default=[1.0, 1.0],
                       help='Goal position (x y)')
    parser.add_argument('--goals', type=str, nargs='+',
                       help='Multiple goals as "x,y" pairs')
    parser.add_argument('--speed', type=float, default=0.25,
                       help='Maximum linear velocity')
    parser.add_argument('--timeout', type=float, default=60.0,
                       help='Navigation timeout per goal')
    parser.add_argument('--no-imu', action='store_true',
                       help='Disable IMU fusion')
    parser.add_argument('--reset', action='store_true',
                       help='Reset odometry before each goal')
    parser.add_argument('--no-stop', action='store_true',
                       help='Don\'t stop between goals')
    
    args = parser.parse_args()
    
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
    config.position_tolerance = 0.12
    config.reset_before_goal = args.reset
    
    try:
        # Get starting position
        state = robot.get_state()
        logger.info(f"Starting at: ({state.x:.2f}, {state.y:.2f})")
        
        results = []
        
        for i, (goal_x, goal_y) in enumerate(goals):
            print(f"\n{'='*50}")
            print(f"GOAL {i+1}/{len(goals)}: ({goal_x:.2f}, {goal_y:.2f})")
            if args.reset:
                print("  Odometry reset ENABLED")
            print('='*50)
            
            # Create new controller for each goal
            gtg = GoToGoal(robot, config)
            
            # Run navigation
            success = gtg.go_to(goal_x, goal_y, callback=status_callback)
            results.append(success)
            
            # Print result
            status = gtg.get_status()
            print(f"\nGoal {i+1} result:")
            print(f"  Position: ({status['x']:.2f}, {status['y']:.2f})")
            print(f"  Distance: {status['distance_to_goal']:.3f}m")
            print(f"  Heading Error: {status['heading_error_deg']:.1f}deg")
            print(f"  Time: {status['elapsed_time']:.1f}s")
            print(f"  Success: {'YES' if success else 'NO'}")
            
            # Wait between goals
            if i < len(goals) - 1 and not args.no_stop and success:
                print(f"\nWaiting 2 seconds before next goal...")
                time.sleep(2.0)
        
        # Summary
        print("\n" + "="*50)
        print("SUMMARY")
        print("="*50)
        for i, (goal, success) in enumerate(zip(goals, results)):
            status = "YES" if success else "NO"
            print(f"Goal {i+1}: ({goal[0]:.2f}, {goal[1]:.2f}) - {status}")
        
        total_success = sum(results)
        print(f"\nTotal: {total_success}/{len(goals)} goals reached")
        print("="*50)
        
        # Final position
        state = robot.get_state()
        print(f"\nFinal position: ({state.x:.2f}, {state.y:.2f})")
        
        return 0 if all(results) else 1
        
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