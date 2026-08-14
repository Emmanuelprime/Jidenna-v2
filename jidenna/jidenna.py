import time
import math
from .serial_comm import SerialComm
from .sensor_fusion import SensorFusion
from .heading_controller import HeadingController

class Jidenna:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.serial = SerialComm(port, baudrate)
        self.fusion = SensorFusion()
        self.heading_controller = HeadingController()
        
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_command_time = 0
        self.command_interval = 0.05
        
        self.is_connected = False
        
    def connect(self):
        self.serial.connect()
        self.is_connected = True
        print(f"Connected to robot on {self.serial.port}")
        
    def disconnect(self):
        self.stop()
        self.serial.disconnect()
        self.is_connected = False
        print("Disconnected from robot")
        
    def stop(self):
        self.serial.send_command(0, 0)
        self.current_v = 0.0
        self.current_w = 0.0
        
    def drive(self, v, w):
        """Send velocity command directly"""
        self.serial.send_command(v, w)
        self.current_v = v
        self.current_w = w
        
    def drive_straight(self, v, duration=None):
        """Drive straight with heading correction"""
        start_time = time.time()
        
        # Wait for initial data and set target heading
        while self.fusion.is_calibrating:
            data = self.serial.get_latest_data()
            if data:
                gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
                fused_heading = self.fusion.update(data['theta'], gyro_rate_rad, data['timestamp'])
            time.sleep(0.01)
        
        while True:
            if duration and (time.time() - start_time) > duration:
                break
                
            data = self.serial.get_latest_data()
            if data:
                # Convert gyro rate to rad/s
                gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
                
                # Update sensor fusion
                fused_heading = self.fusion.update(data['theta'], gyro_rate_rad, data['timestamp'])
                
                if fused_heading is not None:
                    # Compute heading correction
                    dt = 0.05  # Use known command interval
                    w = self.heading_controller.compute(fused_heading, gyro_rate_rad, dt)
                    
                    # Send command
                    self.drive(v, w)
            
            time.sleep(0.005)
        
        self.stop()
        
    def get_pose(self):
        """Get current robot pose"""
        data = self.serial.get_latest_data()
        if data:
            return data['x'], data['y'], data['theta']
        return None
    
    def get_heading(self):
        """Get fused heading estimate"""
        return self.fusion.fused_heading