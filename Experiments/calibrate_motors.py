import serial
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

class MotorCalibration:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.data = []
        
    def connect(self):
        """Connect to the Arduino"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # Wait for Arduino to reset
            # Flush any initial data
            self.ser.reset_input_buffer()
            print(f"Connected to {self.port}")
            return True
        except Exception as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def send_command(self, cmd):
        """Send a command to the Arduino"""
        if self.ser:
            self.ser.write((cmd + '\n').encode())
            time.sleep(0.1)
            # Read and discard any immediate responses
            while self.ser.in_waiting:
                self.ser.readline()
    
    def collect_data(self, pwm_value, duration=3.0):
        """Collect speed data for a given PWM value"""
        if not self.ser:
            return []
        
        # Send the move command
        cmd = f"M{pwm_value},{pwm_value}"
        self.send_command(cmd)
        print(f"  Setting PWM to {pwm_value}...")
        
        # Wait for speed to stabilize
        print(f"  Waiting 3 seconds for stabilization...")
        time.sleep(3.0)
        
        # Collect data
        data_points = []
        start_time = time.time()
        print(f"  Collecting data for {duration} seconds...")
        
        while time.time() - start_time < duration:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                if line and not line.startswith('READY') and not line.startswith('Moving'):
                    # Parse the data: PWM_LEFT,PWM_RIGHT,SPEED_LEFT,SPEED_RIGHT
                    parts = line.split(',')
                    if len(parts) == 4:
                        try:
                            pwm_left = int(parts[0])
                            pwm_right = int(parts[1])
                            speed_left = float(parts[2])
                            speed_right = float(parts[3])
                            
                            # Only collect if PWM matches what we sent
                            if pwm_left == abs(pwm_value) and pwm_right == abs(pwm_value):
                                data_points.append({
                                    'pwm': abs(pwm_value),
                                    'pwm_signed': pwm_value,
                                    'speed_left': speed_left,
                                    'speed_right': speed_right,
                                    'direction': 'forward' if pwm_value > 0 else 'reverse'
                                })
                        except ValueError:
                            pass
            time.sleep(0.01)  # Small delay to prevent CPU overload
        
        return data_points
    
    def run_calibration(self):
        """Run the full calibration sequence"""
        print("\n" + "="*60)
        print("MOTOR PWM TO VELOCITY CALIBRATION")
        print("="*60)
        
        # Test forward directions first
        print("\n--- Testing FORWARD directions (PWM: 5 to 60) ---")
        for pwm in range(5, 61, 5):  # 5, 10, 15, ..., 60
            print(f"\nTesting PWM: +{pwm}")
            points = self.collect_data(pwm, duration=3.0)
            if points:
                self.data.extend(points)
                print(f"  Collected {len(points)} data points")
                # Calculate average speeds
                avg_left = np.mean([p['speed_left'] for p in points])
                avg_right = np.mean([p['speed_right'] for p in points])
                print(f"  Avg Left Speed: {avg_left:.4f} m/s, Avg Right Speed: {avg_right:.4f} m/s")
        
        # Stop motors between directions
        print("\nStopping motors...")
        self.send_command("S")
        time.sleep(2)
        
        # Test reverse directions
        print("\n--- Testing REVERSE directions (PWM: -5 to -60) ---")
        for pwm in range(-5, -61, -5):  # -5, -10, -15, ..., -60
            print(f"\nTesting PWM: {pwm}")
            points = self.collect_data(pwm, duration=3.0)
            if points:
                self.data.extend(points)
                print(f"  Collected {len(points)} data points")
                # Calculate average speeds
                avg_left = np.mean([p['speed_left'] for p in points])
                avg_right = np.mean([p['speed_right'] for p in points])
                print(f"  Avg Left Speed: {avg_left:.4f} m/s, Avg Right Speed: {avg_right:.4f} m/s")
        
        # Stop motors
        print("\nStopping motors...")
        self.send_command("S")
        
        print("\n" + "="*60)
        print("CALIBRATION COMPLETE!")
        print(f"Total data points collected: {len(self.data)}")
        print("="*60)
    
    def save_data(self, filename="motor_calibration_data.csv"):
        """Save collected data to CSV"""
        if not self.data:
            print("No data to save!")
            return
        
        df = pd.DataFrame(self.data)
        df.to_csv(filename, index=False)
        print(f"\nData saved to {filename}")
        return df
    
    def analyze_data(self, df=None):
        """Analyze the data and create plots"""
        if df is None:
            if not self.data:
                print("No data to analyze!")
                return
            df = pd.DataFrame(self.data)
        
        # Calculate average speeds for each PWM value
        summary = df.groupby(['pwm', 'direction']).agg({
            'speed_left': ['mean', 'std'],
            'speed_right': ['mean', 'std'],
            'pwm_signed': 'first'
        }).reset_index()
        
        summary.columns = ['pwm', 'direction', 'speed_left_mean', 'speed_left_std', 
                           'speed_right_mean', 'speed_right_std', 'pwm_signed']
        
        # Fit linear regression for each direction
        print("\n" + "="*60)
        print("LINEAR REGRESSION RESULTS")
        print("="*60)
        
        for direction in ['forward', 'reverse']:
            dir_data = summary[summary['direction'] == direction]
            if len(dir_data) > 1:
                # Left motor
                slope_left, intercept_left, r_value_left, p_value_left, std_err_left = \
                    stats.linregress(dir_data['pwm'], dir_data['speed_left_mean'])
                
                # Right motor
                slope_right, intercept_right, r_value_right, p_value_right, std_err_right = \
                    stats.linregress(dir_data['pwm'], dir_data['speed_right_mean'])
                
                print(f"\n{direction.upper()} DIRECTION:")
                print(f"  Left Motor:  Speed = {slope_left:.4f} * PWM + {intercept_left:.4f}")
                print(f"               R² = {r_value_left**2:.4f}")
                print(f"  Right Motor: Speed = {slope_right:.4f} * PWM + {intercept_right:.4f}")
                print(f"               R² = {r_value_right**2:.4f}")
        
        # Create plots
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Motor PWM to Velocity Relationship', fontsize=16)
        
        # Plot 1: Left motor forward
        ax1 = axes[0, 0]
        forward_data = df[df['direction'] == 'forward']
        if not forward_data.empty:
            # Individual points
            ax1.scatter(forward_data['pwm'], forward_data['speed_left'], 
                       alpha=0.3, label='Data points', s=10)
            # Averages
            fwd_avg = forward_data.groupby('pwm')['speed_left'].mean().reset_index()
            ax1.plot(fwd_avg['pwm'], fwd_avg['speed_left'], 'ro-', 
                    label='Average', linewidth=2, markersize=8)
            ax1.set_xlabel('PWM Value')
            ax1.set_ylabel('Speed (m/s)')
            ax1.set_title('Left Motor - Forward')
            ax1.grid(True, alpha=0.3)
            ax1.legend()
        
        # Plot 2: Left motor reverse
        ax2 = axes[0, 1]
        reverse_data = df[df['direction'] == 'reverse']
        if not reverse_data.empty:
            # Use signed PWM for reverse
            ax2.scatter(reverse_data['pwm_signed'], reverse_data['speed_left'], 
                       alpha=0.3, label='Data points', s=10)
            rev_avg = reverse_data.groupby('pwm_signed')['speed_left'].mean().reset_index()
            ax2.plot(rev_avg['pwm_signed'], rev_avg['speed_left'], 'bo-', 
                    label='Average', linewidth=2, markersize=8)
            ax2.set_xlabel('PWM Value')
            ax2.set_ylabel('Speed (m/s)')
            ax2.set_title('Left Motor - Reverse')
            ax2.grid(True, alpha=0.3)
            ax2.legend()
        
        # Plot 3: Right motor forward
        ax3 = axes[1, 0]
        if not forward_data.empty:
            ax3.scatter(forward_data['pwm'], forward_data['speed_right'], 
                       alpha=0.3, label='Data points', s=10)
            fwd_avg = forward_data.groupby('pwm')['speed_right'].mean().reset_index()
            ax3.plot(fwd_avg['pwm'], fwd_avg['speed_right'], 'ro-', 
                    label='Average', linewidth=2, markersize=8)
            ax3.set_xlabel('PWM Value')
            ax3.set_ylabel('Speed (m/s)')
            ax3.set_title('Right Motor - Forward')
            ax3.grid(True, alpha=0.3)
            ax3.legend()
        
        # Plot 4: Right motor reverse
        ax4 = axes[1, 1]
        if not reverse_data.empty:
            ax4.scatter(reverse_data['pwm_signed'], reverse_data['speed_right'], 
                       alpha=0.3, label='Data points', s=10)
            rev_avg = reverse_data.groupby('pwm_signed')['speed_right'].mean().reset_index()
            ax4.plot(rev_avg['pwm_signed'], rev_avg['speed_right'], 'bo-', 
                    label='Average', linewidth=2, markersize=8)
            ax4.set_xlabel('PWM Value')
            ax4.set_ylabel('Speed (m/s)')
            ax4.set_title('Right Motor - Reverse')
            ax4.grid(True, alpha=0.3)
            ax4.legend()
        
        plt.tight_layout()
        plt.savefig('motor_calibration_plots.png', dpi=150)
        plt.show()
        
        return summary

def main():
    # Configuration
    PORT = 'COM19'  # Change this to your port (e.g., '/dev/ttyUSB0' on Linux)
    BAUDRATE = 115200
    
    # Create calibrator instance
    calibrator = MotorCalibration(PORT, BAUDRATE)
    
    # Connect to Arduino
    if not calibrator.connect():
        print("Failed to connect. Please check:")
        print(f"  - Port: {PORT}")
        print("  - USB cable connection")
        print("  - Arduino is powered on")
        return
    
    # Run calibration
    calibrator.run_calibration()
    
    # Save data
    df = calibrator.save_data()
    
    # Analyze and plot
    if df is not None:
        summary = calibrator.analyze_data(df)
        
        # Save summary
        summary.to_csv('motor_calibration_summary.csv', index=False)
        print("\nSummary saved to motor_calibration_summary.csv")
    
    # Close connection
    calibrator.ser.close()
    print("\nCalibration script finished!")

if __name__ == "__main__":
    main()