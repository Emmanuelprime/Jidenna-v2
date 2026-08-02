"""
Data logging plugin for recording robot data
"""

import csv
import json
import time
import os
from typing import Optional, Dict, Any
from datetime import datetime

from .base_plugin import BasePlugin
import logging

logger = logging.getLogger(__name__)


class LoggingPlugin(BasePlugin):
    """
    Plugin for logging robot data to file
    """
    
    def __init__(self, name: str = "LoggingPlugin", log_dir: str = "logs"):
        super().__init__(name)
        self.log_dir = log_dir
        self.csv_file = None
        self.csv_writer = None
        self.json_log = []
        self.log_interval = 0.1  # seconds
        self.last_log_time = 0
        
    def initialize(self):
        super().initialize()
        
        # Create log directory
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create CSV file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.log_dir, f"robot_data_{timestamp}.csv")
        
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        header = ['timestamp', 'x', 'y', 'yaw', 'linear_velocity', 'angular_velocity',
                  'left_speed', 'right_speed', 'left_pwm', 'right_pwm', 'state']
        self.csv_writer.writerow(header)
        self.csv_file.flush()
        
        logger.info(f"Logging to {csv_path}")
    
    def shutdown(self):
        if self.csv_file:
            self.csv_file.close()
        
        # Save JSON log
        if self.json_log:
            json_path = os.path.join(self.log_dir, f"robot_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
            with open(json_path, 'w') as f:
                json.dump(self.json_log, f, indent=2)
        
        super().shutdown()
    
    def on_telemetry(self, data):
        """Log telemetry data"""
        current_time = time.time()
        if current_time - self.last_log_time < self.log_interval:
            return
        
        self.last_log_time = current_time
        
        # Write to CSV
        row = [
            data.timestamp,
            data.x,
            data.y,
            data.yaw,
            data.linear_velocity,
            data.angular_velocity,
            data.left_speed,
            data.right_speed,
            data.left_pwm,
            data.right_pwm,
            data.state
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        
        # Store in JSON buffer
        self.json_log.append(data.to_dict())
        if len(self.json_log) > 1000:
            self.json_log = self.json_log[-500:]  # Keep last 500
    
    def get_data_file(self) -> str:
        """Get current CSV file path"""
        if self.csv_file:
            return self.csv_file.name
        return None