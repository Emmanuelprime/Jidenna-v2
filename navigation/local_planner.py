import math
import logging
from enum import Enum
from typing import Tuple, Optional, List
from dataclasses import dataclass
from .path import Path
from .pure_pursuit import PurePursuit
from .controller import SpeedController

logger = logging.getLogger(__name__)

class PlannerState(Enum):
    """Local planner states"""
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    GOAL_REACHED = "GOAL_REACHED"
    STOPPED = "STOPPED"
    ERROR = "ERROR"

@dataclass
class PlannerConfig:
    """Configuration for local planner"""
    # Pure Pursuit parameters
    lookahead_distance: float = 0.3
    lookahead_min: float = 0.2
    lookahead_max: float = 0.8
    
    # Speed limits
    max_linear_velocity: float = 0.4
    max_angular_velocity: float = 1.0
    min_linear_velocity: float = 0.0
    
    # Acceleration limits
    max_linear_acceleration: float = 0.3  # m/s² (smooth acceleration)
    max_angular_acceleration: float = 0.5  # rad/s² (smooth turning)
    max_lateral_acceleration: float = 0.3  # m/s² (for turn speed calculation)
    
    # Goal tolerances
    position_tolerance: float = 0.15
    heading_tolerance: float = math.radians(30)
    
    # Safety
    max_curvature: float = 2.0
    goal_slowdown_distance: float = 0.5
    stop_on_path_end: bool = True
    
    # Path tracking
    path_replan_distance: float = 0.3
    
    # Final approach parameters
    final_approach_distance: float = 0.3
    final_approach_speed: float = 0.1
    min_approach_speed: float = 0.03
    
    # IMU/Sensor fusion
    use_imu_heading: bool = True
    imu_heading_weight: float = 0.9  # 90% IMU, 10% odometry
    max_heading_discrepancy: float = math.radians(90)  # Only reject IMU if > 90° off
    
    # Position correction
    position_correction_gain: float = 2.0  # More aggressive correction
    max_cross_track_error: float = 0.5
    correction_start_distance: float = 0.08  # Start correcting earlier
    
    # Control loop
    control_frequency: float = 20.0  # Hz
    control_dt: float = 0.05  # seconds (1/control_frequency)

class LocalPlanner:
    """Local path planner for differential drive robot"""
    
    def __init__(self, config: PlannerConfig = None):
        """
        Initialize local planner
        
        Args:
            config: Planner configuration
        """
        self.config = config if config else PlannerConfig()
        self.pure_pursuit = PurePursuit(
            lookahead_distance=self.config.lookahead_distance,
            lookahead_min=self.config.lookahead_min,
            lookahead_max=self.config.lookahead_max
        )
        self.speed_controller = SpeedController(
            max_linear_velocity=self.config.max_linear_velocity,
            max_angular_velocity=self.config.max_angular_velocity,
            min_linear_velocity=self.config.min_linear_velocity,
            max_lateral_acceleration=self.config.max_lateral_acceleration,
            max_linear_acceleration=self.config.max_linear_acceleration,
            max_angular_acceleration=self.config.max_angular_acceleration
        )
        
        # Internal state
        self.state = PlannerState.IDLE
        self.path: Optional[Path] = None
        self.current_path_index = 0
        self.last_velocity_command = (0.0, 0.0)
        self.last_curvature = 0.0
        self.last_lookahead_point = None
        self.distance_to_goal = float('inf')
        self.cross_track_error = 0.0
        
        # IMU data
        self.last_imu_heading = 0.0
        self.last_odom_heading = 0.0
        self.heading_discrepancy = 0.0
        self.fused_heading = 0.0
        
        # Position correction
        self.position_error = 0.0
        self.correction_active = False
        
        # Safety flags
        self._stop_requested = False
        self._odometry_invalid = False
        self._imu_invalid = False
    
    def set_path(self, path_points: List[Tuple[float, float]]):
        """Set new path to follow"""
        try:
            self.path = Path(path_points)
            self.current_path_index = 0
            self.state = PlannerState.RUNNING
            self._stop_requested = False
            self.correction_active = False
            self.speed_controller.reset()  # Reset acceleration limiting
            logger.info(f"New path set with {len(path_points)} points")
        except ValueError as e:
            logger.error(f"Invalid path: {e}")
            self.state = PlannerState.ERROR
    
    def update(self, robot_pose: Tuple[float, float, float], 
               imu_heading: Optional[float] = None) -> Tuple[float, float]:
        """
        Update planner and compute velocity command
        
        Args:
            robot_pose: (x, y, heading) in world frame (odometry)
            imu_heading: Heading from IMU in radians (optional)
        
        Returns:
            (v, w) velocity command
        """
        # Check if stop requested
        if self._stop_requested:
            self.state = PlannerState.STOPPED
            self.last_velocity_command = (0.0, 0.0)
            self.speed_controller.reset()  # Reset for smooth restart
            return self.last_velocity_command
        
        # Check if we have a valid path
        if not self.path or len(self.path.points) < 2:
            self.state = PlannerState.IDLE
            self.last_velocity_command = (0.0, 0.0)
            return self.last_velocity_command
        
        # Validate robot pose
        if not self._is_valid_pose(robot_pose):
            logger.error("Invalid robot pose")
            self.state = PlannerState.ERROR
            self._odometry_invalid = True
            self.last_velocity_command = (0.0, 0.0)
            return self.last_velocity_command
        
        if self._odometry_invalid:
            logger.warning("Odometry was invalid, but now recovering")
            self._odometry_invalid = False
        
        # Fuse heading from odometry and IMU (IMU primary)
        heading = self._fuse_heading(robot_pose[2], imu_heading)
        
        # Create fused pose
        fused_pose = (robot_pose[0], robot_pose[1], heading)
        
        # Find nearest point on path
        nearest_index, nearest_distance = self.path.get_nearest_point((fused_pose[0], fused_pose[1]))
        self.current_path_index = nearest_index
        
        # Calculate cross-track error
        self.cross_track_error = nearest_distance
        
        # Check if position correction is needed
        self.correction_active = self.cross_track_error > self.config.correction_start_distance
        
        if self.correction_active:
            logger.debug(f"Position correction active: error={self.cross_track_error:.3f}m")
        
        # Get goal point
        goal_point = self.path.points[-1]
        
        # Calculate distance to goal (Euclidean distance to final point)
        self.distance_to_goal = math.sqrt(
            (goal_point[0] - fused_pose[0])**2 + 
            (goal_point[1] - fused_pose[1])**2
        )
        
        # Check if goal reached (position only, heading is secondary)
        if self.distance_to_goal < self.config.position_tolerance:
            self.state = PlannerState.GOAL_REACHED
            self.last_velocity_command = (0.0, 0.0)
            self.speed_controller.reset()  # Reset for next path
            logger.info(f"Goal reached! Distance: {self.distance_to_goal:.3f}m")
            return self.last_velocity_command
        
        # Check if path is too short (remaining path distance)
        remaining_path_distance = self.path.get_remaining_distance(nearest_index)
        if remaining_path_distance < 0.01:
            self.state = PlannerState.GOAL_REACHED
            self.last_velocity_command = (0.0, 0.0)
            self.speed_controller.reset()
            logger.info("Path completed")
            return self.last_velocity_command
        
        # FINAL APPROACH: Direct heading control near goal
        if self.distance_to_goal < self.config.final_approach_distance:
            v, w = self._final_approach(fused_pose, goal_point)
            self.last_curvature = 0.0
            self.last_lookahead_point = goal_point
            self.state = PlannerState.RUNNING
            self.last_velocity_command = (v, w)
            return self.last_velocity_command
        
        # NORMAL PATH FOLLOWING: Pure Pursuit with position correction
        # Select lookahead point
        self.pure_pursuit.update_lookahead_distance(abs(self.last_velocity_command[0]))
        lookahead_point, lookahead_index = self.pure_pursuit.select_lookahead_point(
            self.path, fused_pose, nearest_index
        )
        
        self.last_lookahead_point = lookahead_point
        
        if lookahead_point is None:
            if self.config.stop_on_path_end:
                self.state = PlannerState.STOPPED
                self.last_velocity_command = (0.0, 0.0)
                return self.last_velocity_command
            else:
                lookahead_point = self.path.points[-1]
        
        # Calculate curvature
        curvature = self.pure_pursuit.compute_curvature(fused_pose, lookahead_point)
        
        # Apply position correction if needed
        if self.correction_active:
            # Calculate correction based on cross-track error
            correction = self._calculate_position_correction(
                fused_pose, self.path.points[nearest_index]
            )
            curvature += correction
            logger.debug(f"Applied correction: {correction:.3f} (CTE: {self.cross_track_error:.3f}m)")
        
        self.last_curvature = curvature
        
        # Check curvature safety limit
        max_allowed_curvature = self.config.max_curvature
        if self.correction_active:
            # Allow higher curvature during correction
            max_allowed_curvature *= 1.5
        
        if abs(curvature) > max_allowed_curvature:
            logger.debug(f"Curvature limited: {curvature:.2f} -> {max_allowed_curvature:.2f}")
            curvature = math.copysign(max_allowed_curvature, curvature)
        
        # Calculate desired velocity
        target_velocity = self._calculate_target_velocity()
        
        # Reduce speed during correction
        if self.correction_active:
            target_velocity *= 0.7  # Slow down for correction
        
        # Apply speed control with acceleration limiting
        v, w = self.speed_controller.compute_velocity(
            target_velocity, curvature, self.distance_to_goal,
            self.config.goal_slowdown_distance,
            dt=self.config.control_dt
        )
        
        # Set state
        self.state = PlannerState.RUNNING
        self.last_velocity_command = (v, w)
        
        return self.last_velocity_command
    
    def _calculate_position_correction(self, robot_pose: Tuple[float, float, float], 
                                      nearest_point: Tuple[float, float]) -> float:
        """
        Calculate curvature correction to bring robot back to path
        
        Args:
            robot_pose: (x, y, heading)
            nearest_point: (x, y) nearest point on path
        
        Returns:
            Curvature correction value
        """
        # Calculate vector from robot to nearest point
        dx = nearest_point[0] - robot_pose[0]
        dy = nearest_point[1] - robot_pose[1]
        
        # Transform to robot frame
        local_x = dx * math.cos(robot_pose[2]) + dy * math.sin(robot_pose[2])
        local_y = -dx * math.sin(robot_pose[2]) + dy * math.cos(robot_pose[2])
        
        # Calculate correction curvature
        # Positive local_y means path is to the left
        correction = self.config.position_correction_gain * local_y
        
        # Limit correction magnitude
        max_correction = 1.5  # Maximum correction curvature (increased)
        correction = max(-max_correction, min(correction, max_correction))
        
        return correction
    
    def _fuse_heading(self, odom_heading: float, imu_heading: Optional[float]) -> float:
        """
        Fuse heading with IMU as primary source
        
        IMU is more accurate for heading, especially during turns
        Odometry drifts significantly with each turn
        
        Strategy:
        - Trust IMU 90% by default
        - Only fall back to odometry if IMU seems completely wrong (> 90° off)
        """
        # Store headings
        self.last_odom_heading = odom_heading
        
        # If IMU not available or disabled, use odometry
        if not self.config.use_imu_heading or imu_heading is None:
            self.fused_heading = odom_heading
            return odom_heading
        
        # Store IMU heading
        self.last_imu_heading = imu_heading
        
        # Calculate discrepancy
        self.heading_discrepancy = self._normalize_angle(odom_heading - imu_heading)
        discrepancy_deg = abs(math.degrees(self.heading_discrepancy))
        
        # Use IMU as primary heading source
        if discrepancy_deg > 90:
            # IMU might be completely wrong (very unlikely for MPU6050)
            logger.warning(f"Extreme heading discrepancy: {discrepancy_deg:.1f}° - checking sensors")
            self.fused_heading = odom_heading
        else:
            # Trust IMU heavily (90% IMU, 10% odometry)
            w_odom = 1.0 - self.config.imu_heading_weight  # 0.1
            fused = imu_heading + w_odom * self.heading_discrepancy
            self.fused_heading = self._normalize_angle(fused)
            
            # Log only if discrepancy is large
            if discrepancy_deg > 45:
                logger.debug(f"Heading discrepancy: {discrepancy_deg:.1f}° - trusting IMU")
        
        return self.fused_heading
    
    def _final_approach(self, robot_pose: Tuple[float, float, float], 
                       goal_point: Tuple[float, float]) -> Tuple[float, float]:
        """
        Direct heading control for final approach to goal
        
        Args:
            robot_pose: (x, y, heading)
            goal_point: (x, y) target
        
        Returns:
            (v, w) velocity command
        """
        # Calculate angle to goal
        dx = goal_point[0] - robot_pose[0]
        dy = goal_point[1] - robot_pose[1]
        angle_to_goal = math.atan2(dy, dx)
        
        # Calculate heading error
        heading_error = self._normalize_angle(angle_to_goal - robot_pose[2])
        
        # Simple proportional control for heading
        w = 1.5 * heading_error
        
        # Slow down as we approach goal
        speed_factor = max(0.3, self.distance_to_goal / self.config.final_approach_distance)
        v = self.config.final_approach_speed * speed_factor
        
        # Ensure minimum speed to overcome friction
        v = max(self.config.min_approach_speed, min(v, self.config.final_approach_speed))
        
        # Limit angular velocity
        max_w = 0.5
        w = max(-max_w, min(w, max_w))
        
        logger.debug(f"Final approach: dist={self.distance_to_goal:.3f}m, "
                    f"heading_err={math.degrees(heading_error):.1f}°, "
                    f"v={v:.3f}, w={w:.3f}")
        
        return v, w
    
    def stop(self):
        """Request stop"""
        self._stop_requested = True
        self.state = PlannerState.STOPPED
        self.last_velocity_command = (0.0, 0.0)
        self.speed_controller.reset()  # Reset for smooth restart
        logger.info("Stop requested")
    
    def resume(self):
        """Resume after stop"""
        self._stop_requested = False
        if self.path and self.state != PlannerState.GOAL_REACHED:
            self.state = PlannerState.RUNNING
            logger.info("Planner resumed")
    
    def reset(self):
        """Reset planner state"""
        self.path = None
        self.current_path_index = 0
        self.state = PlannerState.IDLE
        self.last_velocity_command = (0.0, 0.0)
        self.last_curvature = 0.0
        self.last_lookahead_point = None
        self.distance_to_goal = float('inf')
        self.cross_track_error = 0.0
        self._stop_requested = False
        self._odometry_invalid = False
        self._imu_invalid = False
        self.heading_discrepancy = 0.0
        self.fused_heading = 0.0
        self.correction_active = False
        self.speed_controller.reset()
        logger.info("Planner reset")
    
    def get_state(self) -> PlannerState:
        """Get current planner state"""
        return self.state
    
    def get_debug_info(self) -> dict:
        """Get debug information about planner"""
        debug_info = {
            'state': self.state.value,
            'current_path_index': self.current_path_index,
            'distance_to_goal': self.distance_to_goal,
            'cross_track_error': self.cross_track_error,
            'curvature': self.last_curvature,
            'lookahead_point': self.last_lookahead_point,
            'velocity_command': self.last_velocity_command,
            'lookahead_distance': self.pure_pursuit.current_lookahead,
            'heading_discrepancy': self.heading_discrepancy,
            'fused_heading': self.fused_heading,
            'odom_heading': self.last_odom_heading,
            'imu_heading': self.last_imu_heading,
            'correction_active': self.correction_active
        }
        
        # Add speed controller info
        speed_info = self.speed_controller.get_debug_info()
        debug_info.update(speed_info)
        
        return debug_info
    
    def _is_valid_pose(self, pose: Tuple[float, float, float]) -> bool:
        """Check if pose is valid"""
        if len(pose) != 3:
            return False
        return all(math.isfinite(p) for p in pose)
    
    def _estimate_current_speed(self, robot_pose: Tuple[float, float, float]) -> float:
        """Estimate current speed from last velocity command"""
        v, _ = self.last_velocity_command
        return abs(v)
    
    def _calculate_target_velocity(self) -> float:
        """Calculate target velocity based on path context"""
        # Simple approach: use max velocity, but reduce near sharp turns
        target_v = self.config.max_linear_velocity
        
        # Reduce speed if curvature is high
        if abs(self.last_curvature) > 1.0:
            target_v *= 0.5
        elif abs(self.last_curvature) > 0.5:
            target_v *= 0.75
        
        # Reduce speed near goal, but maintain minimum
        if self.distance_to_goal < self.config.goal_slowdown_distance:
            slowdown_factor = max(0.3, self.distance_to_goal / self.config.goal_slowdown_distance)
            target_v *= slowdown_factor
        
        # Reduce speed if cross-track error is large
        if self.cross_track_error > 0.3:
            target_v *= 0.5
        
        return target_v
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle