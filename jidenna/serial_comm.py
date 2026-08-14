import serial
import time
import threading
from collections import deque

class SerialComm:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.data_buffer = deque(maxlen=100)
        self.lock = threading.Lock()
        self.running = False
        self.read_thread = None
        
    def connect(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=1
        )
        time.sleep(5)
        self.ser.reset_input_buffer()
        self.ser.write(b"V0,0\n")
        self.ser.flush()
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        
    def _read_loop(self):
        while self.running:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                if line and ',' in line:
                    values = line.split(',')
                    if len(values) == 8:
                        data = {
                            'x': float(values[0]),
                            'y': float(values[1]),
                            'theta': float(values[2]),
                            'vl': float(values[3]),
                            'vr': float(values[4]),
                            'mpu_angle': float(values[5]),
                            'gyro_rate': float(values[6]),
                            'timestamp': float(values[7])
                        }
                        with self.lock:
                            self.data_buffer.append(data)
            time.sleep(0.001)
    
    def send_command(self, v, w):
        if self.ser:
            command = f"V{v},{w}\n"
            self.ser.write(command.encode())
    
    def get_latest_data(self):
        with self.lock:
            if len(self.data_buffer) > 0:
                return self.data_buffer[-1]
            return None
    
    def get_all_data(self):
        with self.lock:
            return list(self.data_buffer)
    
    def disconnect(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1)
        if self.ser:
            self.ser.write(b"V0,0\n")
            time.sleep(0.1)
            self.ser.close()