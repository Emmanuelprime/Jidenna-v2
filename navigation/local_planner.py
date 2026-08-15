import math
import logging
from enum import Enum
from typing import Tuple, Optional, List
from dataclasses import dataclass
from .path import Path
from .pure_pursuite import PurePursuit
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
    lookahead_distance: float = 0.3  # Reduced from 0.5 to prevent overshooting
    lookahead_min: float = 0.2      # Reduced from 0.3
    lookahead_max: float = 0.8      # Reduced from 1.5
    
    # Speed limits
    max_linear_velocity: float = 0.4  # Reduced from 0.5 for smoother control
    max_angular_velocity: float = 1.0  # rad/s
    min_linear_velocity: float = 0.0  # m/s
    
    # Goal tolerances
    position_tolerance: float = 0.15  # Increased from 0.1 for easier goal reaching
    heading_tolerance: float = math.radians(30)  # Reduced from 60 to be more practical
    
    # Safety
    max_curvature: float = 2.0  # 1/m
    goal_slowdown_distance: float = 0.5  # Reduced from 1.0 to maintain speed longer
    stop_on_path_end: bool = True
    
    # Path tracking
    path_replan_distance: float = 0.3  # Reduced from 0.5

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
            min_linear_velocity=self.config.min_linear_velocity
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
        
        # Safety flags
        self._stop_requested = False
        self._odometry_invalid = False
    
    def set_path(self, path_points: List[Tuple[float, float]]):
        """Set new path to follow"""
        try:
            self.path = Path(path_points)
            self.current_path_index = 0
            self.state = PlannerState.RUNNING
            self._stop_requested = False
            logger.info(f"New path set with {len(path_points)} points")
        except ValueError as e:
            logger.error(f"Invalid path: {e}")
            self.state = PlannerState.ERROR
    
    def update(self, robot_pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """
        Update planner and compute velocity command
        
        Args:
            robot_pose: (x, y, heading) in world frame
        
        Returns:
            (v, w) velocity command
        """
        # Check if stop requested
        if self._stop_requested:
            self.state = PlannerState.STOPPED
            self.last_velocity_command = (0.0, 0.0)
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
        
        # Find nearest point on path
        nearest_index, nearest_distance = self.path.get_nearest_point((robot_pose[0], robot_pose[1]))
        self.current_path_index = nearest_index
        
        # Calculate cross-track error
        self.cross_track_error = nearest_distance
        
        # Get goal point
        goal_point = self.path.points[-1]
        
        # Calculate distance to goal (Euclidean distance to final point)
        self.distance_to_goal = math.sqrt(
            (goal_point[0] - robot_pose[0])**2 + 
            (goal_point[1] - robot_pose[1])**2
        )
        
        # Check if goal reached (position only, heading is secondary)
        if self.distance_to_goal < self.config.position_tolerance:
            self.state = PlannerState.GOAL_REACHED
            self.last_velocity_command = (0.0, 0.0)
            logger.info(f"Goal reached! Distance: {self.distance_to_goal:.3f}m")
            return self.last_velocity_command
        
        # Check if path is too short (remaining path distance)
        remaining_path_distance = self.path.get_remaining_distance(nearest_index)
        if remaining_path_distance < 0.01:
            self.state = PlannerState.GOAL_REACHED
            self.last_velocity_command = (0.0, 0.0)
            logger.info("Path completed")
            return self.last_velocity_command
        
        # Select lookahead point
        self.pure_pursuit.update_lookahead_distance(abs(self.last_velocity_command[0]))
        lookahead_point, lookahead_index = self.pure_pursuit.select_lookahead_point(
            self.path, robot_pose, nearest_index
        )
        
        self.last_lookahead_point = lookahead_point
        
        if lookahead_point is None:
            # Path ended unexpectedly
            if self.config.stop_on_path_end:
                self.state = PlannerState.STOPPED
                self.last_velocity_command = (0.0, 0.0)
                return self.last_velocity_command
            else:
                lookahead_point = self.path.points[-1]
        
        # Calculate curvature
        curvature = self.pure_pursuit.compute_curvature(robot_pose, lookahead_point)
        self.last_curvature = curvature
        
        # Check curvature safety limit
        if abs(curvature) > self.config.max_curvature:
            logger.warning(f"Curvature too high: {curvature:.2f}, limiting")
            curvature = math.copysign(self.config.max_curvature, curvature)
        
        # Calculate desired velocity based on current speed and distance
        target_velocity = self._calculate_target_velocity()
        
        # Apply speed control
        v, w = self.speed_controller.compute_velocity(
            target_velocity, curvature, self.distance_to_goal,
            self.config.goal_slowdown_distance
        )
        
        # If very close to goal, ensure minimum speed to overcome friction
        if self.distance_to_goal < 0.3 and abs(v) < 0.05:
            v = 0.05  # Minimum speed to keep moving toward goal
            # Limit angular velocity when close to goal
            w = max(-0.3, min(w, 0.3))
            logger.debug(f"Near goal - maintaining minimum speed: v={v:.3f}, w={w:.3f}")
        
        # Set state
        self.state = PlannerState.RUNNING
        self.last_velocity_command = (v, w)
        
        return self.last_velocity_command
    
    def stop(self):
        """Request stop"""
        self._stop_requested = True
        self.state = PlannerState.STOPPED
        self.last_velocity_command = (0.0, 0.0)
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
        logger.info("Planner reset")
    
    def get_state(self) -> PlannerState:
        """Get current planner state"""
        return self.state
    
    def get_debug_info(self) -> dict:
        """Get debug information about planner"""
        return {
            'state': self.state.value,
            'current_path_index': self.current_path_index,
            'distance_to_goal': self.distance_to_goal,
            'cross_track_error': self.cross_track_error,
            'curvature': self.last_curvature,
            'lookahead_point': self.last_lookahead_point,
            'velocity_command': self.last_velocity_command,
            'lookahead_distance': self.pure_pursuit.current_lookahead
        }
    
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
        
        return target_v