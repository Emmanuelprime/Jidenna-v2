"""
Basic usage examples for robot controller
Safe test version with proper error handling
"""

import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from jidenna.core.robot_controller import RobotController
from jidenna.plugins.navigation import NavigationPlugin
from jidenna.plugins.obstacle_avoidance import ObstacleAvoidancePlugin
from jidenna.plugins.logging_plugin import LoggingPlugin

def main():
    print("ESP32 Robot Controller - Basic Usage Test")
    print("=" * 60)
    
    # Create controller with context manager
    with RobotController(port="/dev/ttyUSB0") as robot:
        
        # Wait for robot to be ready
        print("\nWaiting for robot to initialize...")
        time.sleep(2)
        
        # Check telemetry
        telemetry = robot.get_telemetry()
        if telemetry:
            print(f"✅ Robot connected - Position: ({telemetry.x:.2f}, {telemetry.y:.2f})")
        else:
            print("⚠️  Connected but no telemetry yet")
        
        # ─── Basic Movement ──────────────────────────────────────────────
        print("\n" + "=" * 60)
        print("Basic Movement Tests")
        print("=" * 60)
        
        # Move forward
        print("\n1. Moving forward at 0.3 m/s for 3 seconds...")
        success = robot.move_forward(0.3, duration=3.0)
        if success:
            print("✅ Forward movement complete")
        else:
            print("❌ Forward movement failed")
        
        # Small pause
        time.sleep(1)
        
        # Turn
        # print("\n2. Turning 45 degrees right...")
        # success = robot.turn_degrees(-45, speed=0.3)
        # if success:
        #     print("✅ Turn complete")
        # else:
        #     print("❌ Turn failed")
        
        # # Small pause
        # time.sleep(1)
        
        # Move backward
        print("\n3. Moving backward at 0.2 m/s for 2 seconds...")
        success = robot.move_backward(0.2, duration=2.0)
        if success:
            print("✅ Backward movement complete")
        else:
            print("❌ Backward movement failed")
        
        # Stop
        robot.stop()
        print("\n✅ Robot stopped")
        
        # ─── Plugin System ──────────────────────────────────────────────
        print("\n" + "=" * 60)
        print("Plugin System Tests")
        print("=" * 60)
        
        # Register plugins
        print("\nRegistering plugins...")
        nav_plugin = NavigationPlugin()
        obs_plugin = ObstacleAvoidancePlugin()
        log_plugin = LoggingPlugin()
        
        success1 = robot.register_plugin(nav_plugin)
        success2 = robot.register_plugin(obs_plugin)
        success3 = robot.register_plugin(log_plugin)
        
        if success1 and success2 and success3:
            print("✅ All plugins registered successfully")
        else:
            print("⚠️  Some plugins failed to register")
        
        # ─── Events ────────────────────────────────────────────────────
        print("\n" + "=" * 60)
        print("Event System Test")
        print("=" * 60)
        
        @robot.on_telemetry
        def handle_telemetry(data):
            print(f"📍 Position: ({data.x:.3f}, {data.y:.3f}) | Yaw: {data.yaw:.1f}°")
        
        @robot.on_state_change
        def handle_state_change(state):
            print(f"🔄 State changed to: {state.name}")
        
        @robot.on_error
        def handle_error(error):
            print(f"⚠️  Error: {error}")
        
        @robot.on_command
        def handle_command(cmd):
            print(f"📤 Command sent: {cmd}")
        
        print("Event handlers registered. Will trigger on next movement.")
        
        # ─── Test Movement with Events ──────────────────────────────────
        print("\n" + "=" * 60)
        print("Testing Movement with Events")
        print("=" * 60)
        
        print("\nMoving forward with events...")
        robot.move_forward(0.2, duration=2.0)
        
        # print("\nTurning with events...")
        # robot.turn_degrees(90, speed=0.3)
        
        print("\nStopping...")
        robot.stop()
        
        # # ─── Navigation Test (Simple) ─────────────────────────────────
        # print("\n" + "=" * 60)
        # print("Navigation Test (Simple)")
        # print("=" * 60)
        
        # print("\nGoing to position (0.5, 0.5)...")
        # print("Note: This requires accurate odometry")
        # success = robot.go_to_position(0.5, 0.5, speed=0.2)
        # if success:
        #     print("✅ Reached target position!")
        # else:
        #     print("⚠️  Could not reach target position")
        
        # # ─── Waypoint Test (Optional - Commented out for safety) ─────
        # print("\n" + "=" * 60)
        # print("Waypoint Navigation (Optional)")
        # print("=" * 60)
        # print("⚠️  This will make the robot follow a square path")
        
        # response = input("\nRun waypoint navigation? (y/n): ")
        # if response.lower() == 'y':
        #     # Define a small square path
        #     waypoints = [
        #         (0.0, 0.0),
        #         (0.5, 0.0),
        #         (0.5, 0.5),
        #         (0.0, 0.5),
        #         (0.0, 0.0)
        #     ]
            
        #     print(f"\nFollowing {len(waypoints)} waypoints...")
        #     nav_plugin.set_waypoints(waypoints)
            
        #     # Wait for navigation to complete with timeout
        #     timeout = 20  # seconds
        #     start_time = time.time()
        #     while nav_plugin.waypoints and nav_plugin.current_waypoint_index < len(nav_plugin.waypoints):
        #         if time.time() - start_time > timeout:
        #             print("⚠️  Navigation timeout")
        #             nav_plugin.stop()
        #             break
        #         print(f"Waypoint {nav_plugin.current_waypoint_index + 1}/{len(nav_plugin.waypoints)}")
        #         time.sleep(2)
            
        #     print("✅ Waypoint navigation complete")
        
        # # ─── Autonomous Mode (Optional) ──────────────────────────────
        # print("\n" + "=" * 60)
        # print("Autonomous Mode (Optional)")
        # print("=" * 60)
        # print("⚠️  This will run a larger autonomous path")
        
        # response = input("\nRun autonomous mode? (y/n): ")
        # if response.lower() == 'y':
        #     # Set autonomous waypoints - bigger loop
        #     autonomous_waypoints = [
        #         (0.0, 0.0),
        #         (1.0, 0.0),
        #         (1.0, 1.0),
        #         (0.0, 1.0),
        #         (0.0, 0.0)
        #     ]
            
        #     nav_plugin.set_waypoints(autonomous_waypoints)
            
        #     print("\nRunning autonomous loop...")
        #     timeout = 30
        #     start_time = time.time()
        #     while nav_plugin.waypoints and nav_plugin.current_waypoint_index < len(nav_plugin.waypoints):
        #         if time.time() - start_time > timeout:
        #             print("⚠️  Autonomous timeout")
        #             nav_plugin.stop()
        #             break
        #         progress = f"{nav_plugin.current_waypoint_index}/{len(nav_plugin.waypoints)}"
        #         print(f"Waypoint progress: {progress}")
        #         time.sleep(2)
            
        #     print("✅ Autonomous run complete!")
        
        # # ─── Final Cleanup ─────────────────────────────────────────────
        # print("\n" + "=" * 60)
        # print("Cleaning Up")
        # print("=" * 60)
        
        robot.stop()
        time.sleep(0.5)
        
        # Print final status
        final_telemetry = robot.get_telemetry()
        if final_telemetry:
            print(f"\nFinal Position: ({final_telemetry.x:.3f}, {final_telemetry.y:.3f})")
            print(f"Final Yaw: {final_telemetry.yaw:.1f}°")
            print(f"Total telemetry packets: {robot.get_telemetry_count()}")
        
        print("\n✅ Test complete! Robot stopped and disconnected.")
        print("Check the logs folder for CSV data.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()