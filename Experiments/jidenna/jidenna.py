import time
import math
from .serial_comm import SerialComm
from .sensor_fusion import SensorFusion
from .heading_controller import HeadingHoldController

class Jidenna:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.serial = SerialComm(port, baudrate)
        self.fusion = SensorFusion()
        self.heading_controller = HeadingHoldController()
        
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
        self.serial.send_command(v, w)
        self.current_v = v
        self.current_w = w
        
    def drive_straight(self, v, duration=None):
        start_time = time.time()
        last_time = time.time()
        
        while self.fusion.is_calibrating:
            data = self.serial.get_latest_data()
            if data:
                gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
                fused_heading = self.fusion.update(data['theta'], gyro_rate_rad, data['timestamp'])
            time.sleep(0.01)
        
        # Set initial target heading once calibrated
        data = self.serial.get_latest_data()
        if data and self.fusion.fused_heading is not None:
            self.heading_controller.set_target(self.fusion.fused_heading)
        
        while True:
            if duration and (time.time() - start_time) > duration:
                break
            
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
                
            data = self.serial.get_latest_data()
            if data:
                gyro_rate_rad = data['gyro_rate'] * math.pi / 180.0
                fused_heading = self.fusion.update(data['theta'], gyro_rate_rad, data['timestamp'])
                
                if fused_heading is not None:
                    w = self.heading_controller.compute(
                        fused_heading, 
                        gyro_rate_rad, 
                        dt,
                        linear_velocity=v
                    )
                    
                    self.drive(v, w)
            
            time.sleep(0.005)
        
        self.stop()
        
    def get_pose(self):
        data = self.serial.get_latest_data()
        if data:
            return data['x'], data['y'], data['theta']
        return None
    
    def get_heading(self):
        return self.fusion.fused_heading