#!/usr/bin/env python3
"""Quick ultrasonic test"""

import time
import sys
sys.path.insert(0, '.')

from jidenna.sensors import UltrasonicAPI
from jidenna import RobotState

# Connect
ultrasonic = UltrasonicAPI(port='/dev/ttyUSB1')
if ultrasonic.connect():
    # Create state
    state = RobotState()
    
    # Wait for data
    data = ultrasonic.wait_for_data(timeout=3.0)
    if data:
        # Update state
        state.update_from_ultrasonic_data(data)
        
        # Print
        print(f"Left: {state.ultrasonic_left:.3f} m")
        print(f"Center: {state.ultrasonic_center:.3f} m")
        print(f"Right: {state.ultrasonic_right:.3f} m")
        print(f"Min: {state.min_ultrasonic_distance:.3f} m")
        print(f"Valid: {state.is_ultrasonic_valid()}")
    
    ultrasonic.disconnect()