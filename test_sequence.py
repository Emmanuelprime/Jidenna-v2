#!/usr/bin/env python3
"""Quick single path test for real robot"""

import time
import argparse
import logging
import sys
import os
import math
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from jidenna import RobotAPI
from navigation import LocalPlanner, PlannerConfig, PlannerState
from main import PathGenerator

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=str, required=True, help='Serial port')
    parser.add_argument('--test', type=str, default='straight', 
                       choices=['straight', 'curve', 'turn', 'square', 'circle', 'custom'])
    parser.add_argument('--speed', type=float, default=0.2, help='Max speed (m/s)')
    parser.add_argument('--distance', type=float, default=1.0, help='Distance (m)')
    args = parser.parse_args()
    
    robot = RobotAPI(port=args.port)
    config = PlannerConfig(
        max_linear_velocity=args.speed,
        max_angular_velocity=args.speed * 2,
        position_tolerance=0.15
    )
    planner = LocalPlanner(config)
    
    # Generate path based on test type
    if args.test == 'straight':
        path = PathGenerator.straight_line((0,0), (args.distance, 0), 20)
    elif args.test == 'curve':
        path = PathGenerator.s_curve((0,0), (args.distance, 0.5), 0.3, 30)
    elif args.test == 'turn':
        path = PathGenerator.right_angle_path(
            (0,0), (0, args.distance), (args.distance, args.distance), 25
        )
    elif args.test == 'square':
        d = args.distance
        path = [(0,0), (d,0), (d,d), (0,d), (0,0)]
    elif args.test == 'circle':
        path = PathGenerator.circle((0,0), args.distance, 40)
    
    logger.info(f"Testing: {args.test}")
    logger.info(f"Speed: {args.speed} m/s")
    logger.info(f"Distance: {args.distance}m")
    
    try:
        if not robot.connect():
            logger.error("Connection failed")
            return
        
        time.sleep(1)
        planner.set_path(path)
        
        input("Press ENTER to start...")
        start_time = time.time()
        
        while time.time() - start_time < 30:
            state = robot.get_state()
            pose = (state.x, state.y, state.heading)
            
            v, w = planner.update(pose)
            robot.set_velocity(v, w)
            
            if planner.state == PlannerState.GOAL_REACHED:
                logger.info("✅ Goal reached!")
                break
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        logger.info("Interrupted")
    finally:
        robot.stop()
        time.sleep(0.2)
        robot.disconnect()
        logger.info("Done")

if __name__ == "__main__":
    main()