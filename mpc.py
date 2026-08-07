#!/usr/bin/env python3
"""
Fast MPC Controller for Differential Drive Robot
Optimized for speed and performance
"""

import numpy as np
import serial
import time
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.interpolate import interp1d
import threading
from dataclasses import dataclass
from typing import Tuple, Optional
import sys
import os

class Mode:
    SIMULATION = 1
    REAL = 2

@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vl: float = 0.0
    vr: float = 0.0
    v: float = 0.0
    w: float = 0.0
    timestamp: float = 0.0

@dataclass
class RobotParams:
    """Robot parameters matching the Arduino code"""
    wheel_diameter: float = 0.165  # m
    wheel_separation: float = 0.521  # m
    max_speed: float = 0.8  # m/s
    max_angular_speed: float = 2.0  # rad/s
    max_acceleration: float = 0.3  # m/s²
    max_deceleration: float = 0.3  # m/s²
    max_angular_acceleration: float = 1.0  # rad/s²
    max_pwm: int = 80
    pulses_per_rev: int = 45
    pwm_freq: int = 1000
    control_interval: float = 0.05  # 50ms
    deadzone_left: int = 10
    deadzone_right: int = 10
    wheel_circumference: float = 0.0
    
    def __post_init__(self):
        self.wheel_circumference = np.pi * self.wheel_diameter

class VelocityProfile:
    """Generate velocity profiles for paths"""
    
    @staticmethod
    def trapezoidal_profile(path_length: float, 
                          max_speed: float, 
                          max_accel: float, 
                          max_decel: float,
                          dt: float) -> np.ndarray:
        """Generate trapezoidal velocity profile"""
        t_accel = max_speed / max_accel
        d_accel = 0.5 * max_accel * t_accel**2
        
        t_decel = max_speed / max_decel
        d_decel = 0.5 * max_decel * t_decel**2
        
        if d_accel + d_decel >= path_length:
            v_peak = np.sqrt(2 * path_length * max_accel * max_decel / (max_accel + max_decel))
            t_accel = v_peak / max_accel
            t_decel = v_peak / max_decel
            t_cruise = 0
        else:
            t_cruise = (path_length - d_accel - d_decel) / max_speed
        
        total_time = t_accel + t_cruise + t_decel
        num_steps = int(total_time / dt) + 1
        
        velocities = np.zeros(num_steps)
        times = np.linspace(0, total_time, num_steps)
        
        for i, t in enumerate(times):
            if t < t_accel:
                velocities[i] = max_accel * t
            elif t < t_accel + t_cruise:
                velocities[i] = max_speed if t_cruise > 0 else v_peak
            else:
                t_decel_elapsed = t - (t_accel + t_cruise)
                if t_cruise > 0:
                    velocities[i] = max_speed - max_decel * t_decel_elapsed
                else:
                    velocities[i] = v_peak - max_decel * t_decel_elapsed
        
        velocities = np.maximum(velocities, 0)
        
        # Light smoothing
        kernel = np.array([0.2, 0.6, 0.2])
        velocities = np.convolve(velocities, kernel, mode='same')
        
        return velocities, times

class EnhancedPathPlanner:
    """Generate reference paths with velocity profiles"""
    
    def __init__(self, params: RobotParams):
        self.params = params
    
    def circle_path_with_profile(self, radius: float = 2.0, num_points: int = 200) -> Tuple[np.ndarray, np.ndarray]:
        """Generate circular path with velocity profile"""
        path_length = 2 * np.pi * radius
        
        v_desired = min(self.params.max_speed * 0.6, 
                       np.sqrt(self.params.max_acceleration * radius))
        w_desired = v_desired / radius
        
        t = np.linspace(0, 2*np.pi, num_points)
        x = radius * np.cos(t)
        y = radius * np.sin(t)
        theta = t + np.pi/2
        
        v_profile, time_profile = VelocityProfile.trapezoidal_profile(
            path_length, v_desired,
            self.params.max_acceleration, self.params.max_deceleration,
            self.params.control_interval
        )
        
        arc_lengths = np.linspace(0, path_length, num_points)
        cumulative_distances = np.cumsum(v_profile) * self.params.control_interval
        if cumulative_distances[-1] > 0:
            cumulative_distances = cumulative_distances / cumulative_distances[-1] * path_length
        
        v_interpolator = interp1d(cumulative_distances, v_profile, 
                                 kind='linear', fill_value='extrapolate')
        v_path = v_interpolator(arc_lengths)
        
        w_path = v_path / radius
        
        path = np.column_stack([x, y, theta])
        velocities = np.column_stack([v_path, w_path])
        
        return path, velocities
    
    def straight_line_with_profile(self, length: float = 5.0, num_points: int = 200) -> Tuple[np.ndarray, np.ndarray]:
        """Generate straight line path with trapezoidal velocity profile"""
        v_profile, time_profile = VelocityProfile.trapezoidal_profile(
            length, self.params.max_speed * 0.7,
            self.params.max_acceleration, self.params.max_deceleration,
            self.params.control_interval
        )
        
        cumulative_distances = np.cumsum(v_profile) * self.params.control_interval
        if cumulative_distances[-1] > 0:
            cumulative_distances = cumulative_distances / cumulative_distances[-1] * length
        
        x = np.linspace(0, length, num_points)
        y = np.zeros_like(x)
        theta = np.zeros_like(x)
        
        v_interpolator = interp1d(cumulative_distances, v_profile, 
                                 kind='linear', fill_value='extrapolate')
        v_path = v_interpolator(np.linspace(0, length, num_points))
        
        path = np.column_stack([x, y, theta])
        velocities = np.column_stack([v_path, np.zeros_like(v_path)])
        
        return path, velocities
    
    def waypoint_path_with_profile(self, waypoints: list, num_points: int = 300) -> Tuple[np.ndarray, np.ndarray]:
        """Generate waypoint path with velocity profile"""
        waypoints = np.array(waypoints)
        
        total_distance = 0
        distances = [0]
        for i in range(1, len(waypoints)):
            dist = np.linalg.norm(waypoints[i] - waypoints[i-1])
            total_distance += dist
            distances.append(total_distance)
        
        t = np.linspace(0, 1, num_points)
        x = np.interp(t, np.array(distances)/total_distance, waypoints[:, 0])
        y = np.interp(t, np.array(distances)/total_distance, waypoints[:, 1])
        
        dx = np.gradient(x)
        dy = np.gradient(y)
        theta = np.arctan2(dy, dx)
        
        v_profile, _ = VelocityProfile.trapezoidal_profile(
            total_distance, self.params.max_speed * 0.5,
            self.params.max_acceleration, self.params.max_deceleration,
            self.params.control_interval
        )
        
        cumulative_distances = np.cumsum(v_profile) * self.params.control_interval
        if cumulative_distances[-1] > 0:
            cumulative_distances = cumulative_distances / cumulative_distances[-1] * total_distance
        
        v_interpolator = interp1d(cumulative_distances, v_profile, 
                                 kind='linear', fill_value='extrapolate')
        v_path = v_interpolator(np.linspace(0, total_distance, num_points))
        
        # ===== TUNING FIX: Less aggressive corner slowdown =====
        for i in range(1, len(theta)-1):
            if abs(theta[i] - theta[i-1]) > 0.1:
                v_path[i-3:i+3] *= 0.6  # Slower decel, higher speed retention
        
        path = np.column_stack([x, y, theta])
        velocities = np.column_stack([v_path, np.zeros_like(v_path)])
        
        return path, velocities

class FastMPCController:
    """Fast MPC Controller with simplified cost function"""
    
    def __init__(self, params: RobotParams, horizon: int = 20):  # ===== TUNING FIX: Horizon increased to 20 =====
        self.params = params
        self.horizon = horizon
        self.dt = params.control_interval
        
        # ===== TUNING FIX: Heavily increased weights for accuracy =====
        self.Q_pos = np.diag([200.0, 300.0])  # Position error weight (was 50, 80)
        self.Q_theta = 50.0                   # Heading error weight (was 20)
        self.Q_vel = np.diag([30.0, 10.0])    # Velocity tracking weight (was 15, 5)
        self.R = np.diag([0.02, 0.05])        # Control effort weight (was 0.1, 0.3)
        self.R_delta = np.diag([0.01, 0.1])   # Control change weight
        
        # Constraints
        self.v_max = params.max_speed * 0.7
        self.w_max = params.max_angular_speed * 0.6
        self.v_accel_max = params.max_acceleration * 0.8
        self.w_accel_max = params.max_angular_acceleration * 0.6
        
        # Previous control
        self.prev_v = 0.0
        self.prev_w = 0.0
        
        # Lookahead
        self.lookahead_steps = 4
        
    def solve(self, current_state: RobotState,
              reference_path: np.ndarray,
              reference_velocities: np.ndarray,
              reference_idx: int) -> Tuple[float, float]:
        """Solve MPC with simplified cost function"""
        
        # Initial guess
        u0 = np.zeros(2 * self.horizon)
        u0[0::2] = self.prev_v
        u0[1::2] = self.prev_w
        
        # Bounds
        bounds = []
        for i in range(self.horizon):
            bounds.extend([
                (-self.v_max, self.v_max),
                (-self.w_max, self.w_max)
            ])
        
        # Solve optimization with faster settings
        try:
            result = minimize(
                self._cost_function_simplified,
                u0,
                args=(current_state, reference_path, reference_velocities, reference_idx),
                bounds=bounds,
                method='L-BFGS-B',
                options={'maxiter': 80, 'ftol': 1e-5}
            )
            
            if result.success:
                u_opt = result.x
                v_opt = u_opt[0]
                w_opt = u_opt[1]
                
                # Apply acceleration limits
                v_opt = np.clip(v_opt, 
                               self.prev_v - self.v_accel_max * self.dt,
                               self.prev_v + self.v_accel_max * self.dt)
                w_opt = np.clip(w_opt,
                               self.prev_w - self.w_accel_max * self.dt,
                               self.prev_w + self.w_accel_max * self.dt)
                
                self.prev_v = v_opt
                self.prev_w = w_opt
                
                return v_opt, w_opt
            else:
                return self._fallback_controller(current_state, reference_path, 
                                                reference_velocities, reference_idx)
                
        except Exception as e:
            return self._fallback_controller(current_state, reference_path,
                                            reference_velocities, reference_idx)
    
    def _cost_function_simplified(self, u: np.ndarray,
                                 current_state: RobotState,
                                 reference_path: np.ndarray,
                                 reference_velocities: np.ndarray,
                                 reference_idx: int) -> float:
        """Simplified cost function for faster computation"""
        cost = 0.0
        state = np.array([current_state.x, current_state.y, current_state.theta, 
                         current_state.v, current_state.w])
        prev_u = np.array([self.prev_v, self.prev_w])
        
        for k in range(self.horizon):
            v_k = u[2*k]
            w_k = u[2*k + 1]
            u_k = np.array([v_k, w_k])
            
            # Predict next state
            state = self._predict_state(state, u_k)
            
            # Get reference
            # ===== TUNING FIX: Cleaned up lookahead logic =====
            ref_idx = min(reference_idx + self.lookahead_steps + k, len(reference_path) - 1)
            ref_pos = reference_path[ref_idx]
            ref_vel = reference_velocities[ref_idx]
            
            # Position error
            pos_error = state[:2] - ref_pos[:2]
            cost += pos_error.T @ self.Q_pos @ pos_error
            
            # Heading error
            theta_error = state[2] - ref_pos[2]
            theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))
            cost += self.Q_theta * theta_error**2
            
            # Velocity error
            vel_error = state[3:5] - ref_vel
            cost += vel_error.T @ self.Q_vel @ vel_error
            
            # Control effort
            cost += u_k.T @ self.R @ u_k
            
            # Control change
            if k == 0:
                delta_u = u_k - prev_u
            else:
                prev_u_k = np.array([u[2*(k-1)], u[2*(k-1)+1]])
                delta_u = u_k - prev_u_k
            cost += delta_u.T @ self.R_delta @ delta_u
        
        return cost
    
    def _predict_state(self, state: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Predict next state"""
        x, y, theta, v, w = state
        v_cmd, w_cmd = u
        
        # Apply velocity dynamics
        tau_v = 0.1
        tau_w = 0.1
        v_next = v + (v_cmd - v) * self.dt / tau_v
        w_next = w + (w_cmd - w) * self.dt / tau_w
        
        # Position update
        x_next = x + v * np.cos(theta) * self.dt
        y_next = y + v * np.sin(theta) * self.dt
        theta_next = theta + w * self.dt
        
        theta_next = np.arctan2(np.sin(theta_next), np.cos(theta_next))
        
        return np.array([x_next, y_next, theta_next, v_next, w_next])
    
    def _fallback_controller(self, current_state: RobotState,
                            reference_path: np.ndarray,
                            reference_velocities: np.ndarray,
                            reference_idx: int) -> Tuple[float, float]:
        """Simple fallback controller"""
        ref_idx = min(reference_idx + 2, len(reference_path) - 1)
        ref_pos = reference_path[ref_idx]
        ref_vel = reference_velocities[ref_idx]
        
        dx = ref_pos[0] - current_state.x
        dy = ref_pos[1] - current_state.y
        
        target_heading = np.arctan2(dy, dx)
        heading_error = target_heading - current_state.theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        distance = np.sqrt(dx**2 + dy**2)
        
        v = ref_vel[0] + 0.5 * distance * np.cos(heading_error)
        w = ref_vel[1] + 1.5 * heading_error
        
        v = np.clip(v, -self.v_max, self.v_max)
        w = np.clip(w, -self.w_max, self.w_max)
        
        v = np.clip(v, self.prev_v - self.v_accel_max * self.dt,
                   self.prev_v + self.v_accel_max * self.dt)
        w = np.clip(w, self.prev_w - self.w_accel_max * self.dt,
                   self.prev_w + self.w_accel_max * self.dt)
        
        self.prev_v = v
        self.prev_w = w
        
        return v, w

class RobotSimulator:
    """Robot simulator"""
    
    def __init__(self, params: RobotParams):
        self.params = params
        self.state = RobotState()
        self.current_vl = 0.0
        self.current_vr = 0.0
        # ===== TUNING FIX: Faster motor response in simulation =====
        self.tau_motor = 0.04  # Was 0.1
        
        self.log = {
            'time': [], 'x': [], 'y': [], 'theta': [],
            'vl': [], 'vr': [], 'v': [], 'w': [],
            'v_cmd': [], 'w_cmd': []
        }
    
    def set_velocity(self, v: float, w: float):
        L = self.params.wheel_separation
        target_vl = v - (w * L) / 2.0
        target_vr = v + (w * L) / 2.0
        
        target_vl = np.clip(target_vl, -self.params.max_speed, self.params.max_speed)
        target_vr = np.clip(target_vr, -self.params.max_speed, self.params.max_speed)
        
        self.current_vl = self._motor_dynamics(self.current_vl, target_vl)
        self.current_vr = self._motor_dynamics(self.current_vr, target_vr)
        
        self.log['v_cmd'].append(v)
        self.log['w_cmd'].append(w)
    
    def _motor_dynamics(self, current: float, target: float) -> float:
        if abs(target) < 0.01:
            target = 0.0
        
        alpha = self.params.control_interval / self.tau_motor
        new_velocity = current + alpha * (target - current)
        return new_velocity
    
    def update(self, dt: float) -> RobotState:
        L = self.params.wheel_separation
        v_linear = (self.current_vl + self.current_vr) / 2.0
        v_angular = (self.current_vr - self.current_vl) / L
        
        self.state.x += v_linear * np.cos(self.state.theta) * dt
        self.state.y += v_linear * np.sin(self.state.theta) * dt
        self.state.theta += v_angular * dt
        self.state.vl = self.current_vl
        self.state.vr = self.current_vr
        self.state.v = v_linear
        self.state.w = v_angular
        self.state.timestamp += dt
        
        self.state.theta = np.arctan2(np.sin(self.state.theta), np.cos(self.state.theta))
        
        self.log['time'].append(self.state.timestamp)
        self.log['x'].append(self.state.x)
        self.log['y'].append(self.state.y)
        self.log['theta'].append(self.state.theta)
        self.log['vl'].append(self.current_vl)
        self.log['vr'].append(self.current_vr)
        self.log['v'].append(v_linear)
        self.log['w'].append(v_angular)
        
        return self.state

class RealRobot:
    """Interface with the real Arduino robot"""
    
    def __init__(self, port: str = None, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.state = RobotState()
        self.lock = threading.Lock()
        self._running = False
        self._reader_thread = None
        
        self.log = {
            'time': [], 'x': [], 'y': [], 'theta': [],
            'vl': [], 'vr': [], 'v': [], 'w': [],
            'v_cmd': [], 'w_cmd': []
        }
        
    def connect(self) -> bool:
        if self.port is None:
            if sys.platform.startswith('win'):
                self.port = self._find_windows_port()
            else:
                self.port = '/dev/ttyUSB0'
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            print(f"Connecting to robot on {self.port}...")
            time.sleep(2)
            
            timeout = time.time() + 5
            while time.time() < timeout:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line == "READY":
                        print("Robot connected and ready!")
                        break
            else:
                print("Timeout waiting for robot READY signal")
                return False
            
            self._running = True
            self._reader_thread = threading.Thread(target=self._read_serial)
            self._reader_thread.daemon = True
            self._reader_thread.start()
            return True
            
        except serial.SerialException as e:
            print(f"Failed to connect to robot: {e}")
            return False
    
    def _find_windows_port(self) -> str:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if ports:
            print("Available ports:")
            for i, port in enumerate(ports):
                print(f"  {i+1}. {port.device} - {port.description}")
            while True:
                try:
                    choice = input(f"Select port (1-{len(ports)}): ").strip()
                    idx = int(choice) - 1
                    if 0 <= idx < len(ports):
                        return ports[idx].device
                except ValueError:
                    pass
                print("Invalid selection. Try again.")
        else:
            port = input("No ports found. Enter COM port manually (e.g., COM3): ").strip()
            return port
    
    def _read_serial(self):
        while self._running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if ',' in line and not any(line.startswith(x) for x in ['READY', 'POSE', 'YAW']):
                        try:
                            parts = line.split(',')
                            if len(parts) >= 7:
                                with self.lock:
                                    self.state.x = float(parts[0])
                                    self.state.y = float(parts[1])
                                    self.state.theta = float(parts[2])
                                    self.state.vl = float(parts[3])
                                    self.state.vr = float(parts[4])
                                    self.state.v = float(parts[5])
                                    self.state.w = float(parts[6])
                                    self.state.timestamp = time.time()
                                    
                                    self.log['time'].append(self.state.timestamp)
                                    self.log['x'].append(self.state.x)
                                    self.log['y'].append(self.state.y)
                                    self.log['theta'].append(self.state.theta)
                                    self.log['vl'].append(self.state.vl)
                                    self.log['vr'].append(self.state.vr)
                                    self.log['v'].append(self.state.v)
                                    self.log['w'].append(self.state.w)
                        except (ValueError, IndexError):
                            pass
            except serial.SerialException:
                self._running = False
                break
            except Exception as e:
                print(f"Serial read error: {e}")
    
    def send_command(self, v: float, w: float):
        if self.ser and self.ser.is_open:
            cmd = f"D{v:.4f},{w:.4f}\n"
            try:
                self.ser.write(cmd.encode('utf-8'))
                self.log['v_cmd'].append(v)
                self.log['w_cmd'].append(w)
            except serial.SerialException as e:
                print(f"Failed to send command: {e}")
    
    def get_state(self) -> RobotState:
        with self.lock:
            return RobotState(
                x=self.state.x, y=self.state.y, theta=self.state.theta,
                vl=self.state.vl, vr=self.state.vr,
                v=self.state.v, w=self.state.w,
                timestamp=self.state.timestamp
            )
    
    def disconnect(self):
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        
        if self.ser and self.ser.is_open:
            self.ser.write(b'S\n')
            time.sleep(0.1)
            self.ser.close()
            print("Robot disconnected")

def plot_results(robot, reference_path, reference_velocities, mode_name):
    """Plot results"""
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    
    # Plot 1: Trajectory
    ax1 = axes[0, 0]
    ax1.plot(reference_path[:, 0], reference_path[:, 1], 'g--', linewidth=2, label='Reference Path')
    ax1.plot(robot.log['x'], robot.log['y'], 'b-', linewidth=2, label='Actual Trajectory')
    ax1.plot(robot.log['x'][0], robot.log['y'][0], 'go', markersize=10, label='Start')
    ax1.plot(robot.log['x'][-1], robot.log['y'][-1], 'ro', markersize=10, label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title(f'Robot Trajectory - {mode_name}')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # Plot 2: Position over time
    ax2 = axes[0, 1]
    time_array = np.array(robot.log['time']) - robot.log['time'][0]
    ax2.plot(time_array, robot.log['x'], 'b-', label='X position')
    ax2.plot(time_array, robot.log['y'], 'r-', label='Y position')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Linear Velocity
    ax3 = axes[0, 2]
    cmd_time = np.arange(len(robot.log['v_cmd'])) * 0.05
    ax3.plot(cmd_time, robot.log['v_cmd'], 'b-', linewidth=2, label='Commanded V')
    ax3.plot(time_array, robot.log['v'], 'r-', linewidth=2, label='Actual V', alpha=0.7)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Linear Velocity (m/s)')
    ax3.set_title('Linear Velocity')
    ax3.legend()
    ax3.grid(True)
    
    # Plot 4: Angular Velocity
    ax4 = axes[1, 0]
    ax4.plot(cmd_time, robot.log['w_cmd'], 'b-', linewidth=2, label='Commanded W')
    ax4.plot(time_array, robot.log['w'], 'r-', linewidth=2, label='Actual W', alpha=0.7)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Angular Velocity (rad/s)')
    ax4.set_title('Angular Velocity')
    ax4.legend()
    ax4.grid(True)
    
    # Plot 5: Wheel Velocities
    ax5 = axes[1, 1]
    ax5.plot(time_array, robot.log['vl'], 'b-', label='Left Wheel', alpha=0.7)
    ax5.plot(time_array, robot.log['vr'], 'r-', label='Right Wheel', alpha=0.7)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Velocity (m/s)')
    ax5.set_title('Wheel Velocities')
    ax5.legend()
    ax5.grid(True)
    
    # Plot 6: Heading
    ax6 = axes[1, 2]
    ax6.plot(time_array, np.degrees(robot.log['theta']), 'b-', linewidth=2)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Heading (degrees)')
    ax6.set_title('Robot Heading')
    ax6.grid(True)
    
    plt.tight_layout()
    plt.show()

def get_user_input():
    """Get user input"""
    print("\n" + "="*60)
    print("FAST MPC ROBOT CONTROLLER")
    print("="*60)
    
    print("\nSelect operation mode:")
    print("  1. Simulation Mode")
    print("  2. Real Robot Mode")
    
    while True:
        try:
            mode_choice = input("\nEnter choice (1 or 2): ").strip()
            if mode_choice in ['1', '2']:
                mode = int(mode_choice)
                break
            else:
                print("Invalid choice. Please enter 1 or 2.")
        except:
            print("Invalid input. Please try again.")
    
    print("\nSelect reference path:")
    print("  1. Circle (radius 2m)")
    print("  2. Straight Line (5m)")
    print("  3. Square Waypoints")
    
    while True:
        try:
            path_choice = input("\nEnter choice (1-3): ").strip()
            if path_choice in ['1', '2', '3']:
                path_type = int(path_choice)
                break
            else:
                print("Invalid choice. Please enter 1-3.")
        except:
            print("Invalid input. Please try again.")
    
    return mode, path_type

def main():
    """Main function"""
    
    mode, path_type = get_user_input()
    
    params = RobotParams()
    path_planner = EnhancedPathPlanner(params)
    
    # ===== TUNING FIX: Increased MPC horizon =====
    mpc = FastMPCController(params, horizon=20)
    
    # Generate path
    if path_type == 1:
        reference_path, reference_velocities = path_planner.circle_path_with_profile(radius=2.0)
        path_name = "Circle"
    elif path_type == 2:
        reference_path, reference_velocities = path_planner.straight_line_with_profile(length=5.0)
        path_name = "Straight Line"
    elif path_type == 3:
        waypoints = [[0, 0], [2, 0], [2, 2], [0, 2], [0, 0]]
        reference_path, reference_velocities = path_planner.waypoint_path_with_profile(waypoints)
        path_name = "Square Waypoints"
    
    print(f"\nSelected path: {path_name}")
    print(f"Path length: {len(reference_path)} points")
    
    # Initialize robot
    if mode == Mode.REAL:
        robot = RealRobot()
        if not robot.connect():
            print("Failed to connect to robot. Exiting.")
            return
        mode_name = "REAL ROBOT"
    else:
        robot = RobotSimulator(params)
        mode_name = "SIMULATION"
    
    print(f"\nRunning in {mode_name} mode with {path_name}")
    print("Press Ctrl+C to stop early\n")
    
    current_state = RobotState()
    reference_idx = 0
    step = 0
    
    try:
        while True:
            if isinstance(robot, RobotSimulator):
                current_state = robot.update(params.control_interval)
            else:
                current_state = robot.get_state()
            
            # Find closest reference point
            distances = np.linalg.norm(
                reference_path[:, :2] - [current_state.x, current_state.y], 
                axis=1
            )
            reference_idx = np.argmin(distances)
            
            # Check completion
            if reference_idx >= len(reference_path) - 3:
                print("\nPath completed!")
                break
            
            # Look ahead
            lookahead = min(reference_idx + 4, len(reference_path) - 1)
            
            # Solve MPC
            v_opt, w_opt = mpc.solve(current_state, reference_path, 
                                    reference_velocities, lookahead)
            
            # Apply control
            if isinstance(robot, RobotSimulator):
                robot.set_velocity(v_opt, w_opt)
            else:
                robot.send_command(v_opt, w_opt)
            
            # Print status
            if step % 20 == 0:
                ref_vel = reference_velocities[reference_idx]
                print(f"Step {step:3d}: x={current_state.x:6.2f}, y={current_state.y:6.2f}, "
                      f"v={current_state.v:5.2f} (ref:{ref_vel[0]:.2f}), "
                      f"w={current_state.w:5.2f} (ref:{ref_vel[1]:.2f})")
            
            step += 1
            time.sleep(params.control_interval)
    
    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    
    finally:
        if isinstance(robot, RealRobot):
            robot.disconnect()
        
        print("\nGenerating results plot...")
        plot_results(robot, reference_path, reference_velocities, mode_name)
        
        if len(robot.log['x']) > 0:
            print("\n" + "="*60)
            print("PERFORMANCE STATISTICS")
            print("="*60)
            final_distance = np.linalg.norm([robot.log['x'][-1] - reference_path[-1, 0],
                                             robot.log['y'][-1] - reference_path[-1, 1]])
            total_time = robot.log['time'][-1] - robot.log['time'][0]
            
            lateral_errors = np.abs(np.array(robot.log['y']) - reference_path[0, 1])
            
            print(f"Total time: {total_time:.2f} seconds")
            print(f"Final position error: {final_distance:.3f} meters")
            print(f"Max lateral deviation: {np.max(lateral_errors):.3f} meters")
            print(f"Max velocity achieved: {np.max(robot.log['v']):.3f} m/s")
            print(f"Total steps: {step}")
        
        print("\nController stopped successfully!")

if __name__ == "__main__":
    main()