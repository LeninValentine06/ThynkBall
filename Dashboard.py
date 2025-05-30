#!/usr/bin/env python3
"""
Real-time IMU Data Visualizer
Reads BMI088 + Madgwick filter data from Arduino and displays:
- 3D orientation visualization 
- Real-time sensor plots
- Euler angles display
"""

import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Rectangle
import time
from collections import deque
import threading
import queue

class IMUVisualizer:
    def __init__(self, port='COM3', baudrate=115200, buffer_size=100):
        """
        Initialize the IMU visualizer
        
        Args:
            port: Serial port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
            baudrate: Serial communication speed
            buffer_size: Number of data points to keep in memory
        """
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        
        # Data storage
        self.data_queue = queue.Queue()
        self.timestamps = deque(maxlen=buffer_size)
        self.accel_data = {'x': deque(maxlen=buffer_size), 
                          'y': deque(maxlen=buffer_size), 
                          'z': deque(maxlen=buffer_size)}
        self.gyro_data = {'x': deque(maxlen=buffer_size), 
                         'y': deque(maxlen=buffer_size), 
                         'z': deque(maxlen=buffer_size)}
        self.euler_data = {'roll': deque(maxlen=buffer_size),
                          'pitch': deque(maxlen=buffer_size),
                          'yaw': deque(maxlen=buffer_size)}
        self.quaternion = {'q0': 1, 'q1': 0, 'q2': 0, 'q3': 0}
        self.temperature = 0
        
        # Current orientation
        self.current_roll = 0
        self.current_pitch = 0
        self.current_yaw = 0
        
        # Serial connection
        self.serial_conn = None
        self.running = False
        
    def connect_serial(self):
        """Connect to Arduino via serial"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
    
    def read_serial_data(self):
        """Read data from serial port in separate thread"""
        start_time = time.time()
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    
                    # Skip header lines
                    if line.startswith('BMI088') or line.startswith('ax(') or line.startswith('BMI088 is'):
                        continue
                    
                    # Parse CSV data
                    try:
                        data = [float(x) for x in line.split(',')]
                        if len(data) >= 14:  # Ensure we have all expected data (14 values total)
                            timestamp = time.time() - start_time
                            
                            # Store data according to new format:
                            # ax, ay, az, gx, gy, gz, temp, roll, pitch, yaw, q0, q1, q2, q3
                            data_dict = {
                                'timestamp': timestamp,
                                'ax': data[0], 'ay': data[1], 'az': data[2],
                                'gx': data[3], 'gy': data[4], 'gz': data[5],
                                'temp': data[6],
                                'roll': data[7], 'pitch': data[8], 'yaw': data[9],
                                'q0': data[10], 'q1': data[11], 'q2': data[12], 'q3': data[13]
                            }
                            
                            self.data_queue.put(data_dict)
                            
                    except (ValueError, IndexError):
                        continue  # Skip malformed lines
                        
            except Exception as e:
                print(f"Serial read error: {e}")
                break
                
            time.sleep(0.01)  # Small delay to prevent excessive CPU usage
    
    def update_data_buffers(self):
        """Update internal data buffers from queue"""
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                
                # Update buffers
                self.timestamps.append(data['timestamp'])
                self.accel_data['x'].append(data['ax'])
                self.accel_data['y'].append(data['ay'])
                self.accel_data['z'].append(data['az'])
                self.gyro_data['x'].append(data['gx'])
                self.gyro_data['y'].append(data['gy'])
                self.gyro_data['z'].append(data['gz'])
                self.euler_data['roll'].append(data['roll'])
                self.euler_data['pitch'].append(data['pitch'])
                self.euler_data['yaw'].append(data['yaw'])
                
                # Update current values
                self.current_roll = data['roll']
                self.current_pitch = data['pitch']
                self.current_yaw = data['yaw']
                self.quaternion = {'q0': data['q0'], 'q1': data['q1'], 
                                 'q2': data['q2'], 'q3': data['q3']}
                self.temperature = data['temp']
                
            except queue.Empty:
                break
    
    def quaternion_to_rotation_matrix(self, q0, q1, q2, q3):
        """Convert quaternion to rotation matrix"""
        # Normalize quaternion
        norm = np.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
        q0, q1, q2, q3 = q0/norm, q1/norm, q2/norm, q3/norm
        
        # Rotation matrix
        R = np.array([
            [1-2*(q2**2+q3**2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
            [2*(q1*q2+q0*q3), 1-2*(q1**2+q3**2), 2*(q2*q3-q0*q1)],
            [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1**2+q2**2)]
        ])
        return R
    
    def draw_3d_object(self, ax, R):
        """Draw a 3D object (airplane-like) showing orientation"""
        ax.clear()
        
        # Define airplane vertices (relative to body frame)
        # Main body
        body = np.array([
            [2, 0, 0],    # Nose
            [-1, 0, 0],   # Tail
        ])
        
        # Wings
        wings = np.array([
            [0, -1.5, 0], # Left wing tip
            [0, 1.5, 0],  # Right wing tip
            [0, 0, 0],    # Wing center
        ])
        
        # Vertical stabilizer
        vstab = np.array([
            [-1, 0, 0],   # Tail
            [-1, 0, 0.8], # Top of vertical stabilizer
        ])
        
        # Apply rotation matrix
        body_rot = (R @ body.T).T
        wings_rot = (R @ wings.T).T
        vstab_rot = (R @ vstab.T).T
        
        # Plot body
        ax.plot(body_rot[:, 0], body_rot[:, 1], body_rot[:, 2], 'b-', linewidth=3, label='Body')
        
        # Plot wings
        ax.plot([wings_rot[0, 0], wings_rot[2, 0], wings_rot[1, 0]], 
                [wings_rot[0, 1], wings_rot[2, 1], wings_rot[1, 1]], 
                [wings_rot[0, 2], wings_rot[2, 2], wings_rot[1, 2]], 'g-', linewidth=2, label='Wings')
        
        # Plot vertical stabilizer
        ax.plot(vstab_rot[:, 0], vstab_rot[:, 1], vstab_rot[:, 2], 'r-', linewidth=2, label='V-Stab')
        
        # Draw coordinate axes
        axes_length = 1.0
        origin = np.array([0, 0, 0])
        
        # X-axis (red)
        x_axis = (R @ np.array([axes_length, 0, 0]))
        ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], 'r-', alpha=0.7)
        
        # Y-axis (green)
        y_axis = (R @ np.array([0, axes_length, 0]))
        ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], 'g-', alpha=0.7)
        
        # Z-axis (blue)
        z_axis = (R @ np.array([0, 0, axes_length]))
        ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], 'b-', alpha=0.7)
        
        # Set equal aspect ratio and limits
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_zlim(-2, 2)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('IMU Orientation (3D)')
        
        # Add text display
        ax.text2D(0.02, 0.98, f'Roll: {self.current_roll:.1f}°', transform=ax.transAxes, 
                 verticalalignment='top', fontsize=10, bbox=dict(boxstyle="round", facecolor='wheat'))
        ax.text2D(0.02, 0.90, f'Pitch: {self.current_pitch:.1f}°', transform=ax.transAxes, 
                 verticalalignment='top', fontsize=10, bbox=dict(boxstyle="round", facecolor='lightblue'))
        ax.text2D(0.02, 0.82, f'Yaw: {self.current_yaw:.1f}°', transform=ax.transAxes, 
                 verticalalignment='top', fontsize=10, bbox=dict(boxstyle="round", facecolor='lightgreen'))
        ax.text2D(0.02, 0.74, f'Temp: {self.temperature:.1f}°C', transform=ax.transAxes, 
                 verticalalignment='top', fontsize=10, bbox=dict(boxstyle="round", facecolor='pink'))
    
    def update_plots(self, frame):
        """Update all plots (called by animation)"""
        self.update_data_buffers()
        
        if len(self.timestamps) == 0:
            return
        
        # Get current rotation matrix
        R = self.quaternion_to_rotation_matrix(
            self.quaternion['q0'], self.quaternion['q1'], 
            self.quaternion['q2'], self.quaternion['q3']
        )
        
        # Update 3D orientation plot
        self.draw_3d_object(self.ax3d, R)
        
        # Update sensor plots
        times = list(self.timestamps)
        if len(times) > 1:
            # Accelerometer plot
            self.ax_accel.clear()
            self.ax_accel.plot(times, list(self.accel_data['x']), 'r-', label='Ax', alpha=0.8)
            self.ax_accel.plot(times, list(self.accel_data['y']), 'g-', label='Ay', alpha=0.8)
            self.ax_accel.plot(times, list(self.accel_data['z']), 'b-', label='Az', alpha=0.8)
            self.ax_accel.set_ylabel('Acceleration (m/s²)')
            self.ax_accel.legend(loc='upper right')
            self.ax_accel.grid(True, alpha=0.3)
            self.ax_accel.set_title('Accelerometer Data')
            
            # Gyroscope plot
            self.ax_gyro.clear()
            self.ax_gyro.plot(times, list(self.gyro_data['x']), 'r-', label='Gx', alpha=0.8)
            self.ax_gyro.plot(times, list(self.gyro_data['y']), 'g-', label='Gy', alpha=0.8)
            self.ax_gyro.plot(times, list(self.gyro_data['z']), 'b-', label='Gz', alpha=0.8)
            self.ax_gyro.set_ylabel('Angular Velocity (rad/s)')
            self.ax_gyro.legend(loc='upper right')
            self.ax_gyro.grid(True, alpha=0.3)
            self.ax_gyro.set_title('Gyroscope Data')
            
            # Euler angles plot
            self.ax_euler.clear()
            self.ax_euler.plot(times, list(self.euler_data['roll']), 'r-', label='Roll', alpha=0.8)
            self.ax_euler.plot(times, list(self.euler_data['pitch']), 'g-', label='Pitch', alpha=0.8)
            self.ax_euler.plot(times, list(self.euler_data['yaw']), 'b-', label='Yaw', alpha=0.8)
            self.ax_euler.set_ylabel('Angle (°)')
            self.ax_euler.set_xlabel('Time (s)')
            self.ax_euler.legend(loc='upper right')
            self.ax_euler.grid(True, alpha=0.3)
            self.ax_euler.set_title('Orientation (Euler Angles)')
    
    def run(self):
        """Main visualization loop"""
        if not self.connect_serial():
            return
        
        # Set up the plot
        self.fig = plt.figure(figsize=(15, 10))
        
        # Create subplots
        self.ax3d = self.fig.add_subplot(2, 2, 1, projection='3d')
        self.ax_accel = self.fig.add_subplot(2, 2, 2)
        self.ax_gyro = self.fig.add_subplot(2, 2, 3)
        self.ax_euler = self.fig.add_subplot(2, 2, 4)
        
        plt.tight_layout()
        
        # Start serial reading thread
        self.running = True
        serial_thread = threading.Thread(target=self.read_serial_data)
        serial_thread.daemon = True
        serial_thread.start()
        
        # Start animation
        ani = FuncAnimation(self.fig, self.update_plots, interval=50, blit=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("Stopping visualization...")
        finally:
            self.running = False
            if self.serial_conn:
                self.serial_conn.close()


def main():
    """Main function"""
    print("IMU Real-time Visualizer")
    print("Make sure your Arduino is connected and running the BMI088+Madgwick code")
    
    # Configure your serial port here
    port = input("Enter serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux): ").strip()
    if not port:
        port = "COM3"  # Default for Windows
    
    visualizer = IMUVisualizer(port=port, baudrate=115200)
    visualizer.run()


if __name__ == "__main__":
    main()