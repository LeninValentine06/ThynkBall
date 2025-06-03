import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import serial
import threading
import queue
from datetime import datetime
import argparse
import os
from mpl_toolkits.mplot3d import Axes3D

class EnhancedThynkballVisualizer:
    def __init__(self, serial_port=None, baudrate=115200, csv_file=None, real_time=True):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.csv_file = csv_file
        self.real_time = real_time
        
        # Data storage
        self.data_queue = queue.Queue()
        self.df = pd.DataFrame()
        self.max_points = 1000  # Maximum points to display
        
        # Serial connection
        self.ser = None
        self.running = False
        
        # Enhanced column names for the new CSV data format
        self.columns = [
            'timestamp', 'raw_acc_x', 'raw_acc_y', 'raw_acc_z',
            'g_acc_x', 'g_acc_y', 'g_acc_z', 'raw_gx', 'raw_gy', 'raw_gz',
            'temp', 'roll', 'pitch', 'yaw', 'q0', 'q1', 'q2', 'q3',
            'vx', 'vy', 'vz', 'vx_filt', 'vy_filt', 'vz_filt',
            'vx_world', 'vy_world', 'vz_world',
            'pos_x_world', 'pos_y_world', 'pos_z_world',
            'pos_x_body', 'pos_y_body', 'pos_z_body',
            'total_dist', 'max_height', 'min_height', 'stationary'
        ]
        
        # Setup plots
        self.setup_plots()
        
    def setup_plots(self):
        """Setup the matplotlib figures and subplots with enhanced layout"""
        # Create figure with subplots
        self.fig = plt.figure(figsize=(20, 16))
        self.fig.suptitle('Enhanced Thynkball IMU & Position Visualization', fontsize=18, fontweight='bold')
        
        # Create enhanced subplot grid
        gs = self.fig.add_gridspec(5, 4, hspace=0.3, wspace=0.3)
        
        # Row 1: Sensor Data
        self.ax_acc_comp = self.fig.add_subplot(gs[0, 0])
        self.ax_gyro = self.fig.add_subplot(gs[0, 1])
        self.ax_euler = self.fig.add_subplot(gs[0, 2])
        self.ax_temp_stat = self.fig.add_subplot(gs[0, 3])
        
        # Row 2: Velocity Analysis
        self.ax_vel_body = self.fig.add_subplot(gs[1, 0])
        self.ax_vel_world = self.fig.add_subplot(gs[1, 1])
        self.ax_vel_comparison = self.fig.add_subplot(gs[1, 2])
        self.ax_vel_magnitude = self.fig.add_subplot(gs[1, 3])
        
        # Row 3: Position Tracking
        self.ax_pos_world = self.fig.add_subplot(gs[2, 0])
        self.ax_pos_comparison = self.fig.add_subplot(gs[2, 1])
        self.ax_trajectory_2d = self.fig.add_subplot(gs[2, 2])
        self.ax_height_profile = self.fig.add_subplot(gs[2, 3])
        
        # Row 4: 3D Visualization
        self.ax_3d_trajectory = self.fig.add_subplot(gs[3, :2], projection='3d')
        self.ax_stats = self.fig.add_subplot(gs[3, 2])
        self.ax_motion_analysis = self.fig.add_subplot(gs[3, 3])
        
        # Row 5: Advanced Analysis
        self.ax_velocity_vectors = self.fig.add_subplot(gs[4, 0])
        self.ax_acceleration_world = self.fig.add_subplot(gs[4, 1])
        self.ax_orientation_path = self.fig.add_subplot(gs[4, 2])
        self.ax_energy_analysis = self.fig.add_subplot(gs[4, 3])
        
        # Configure all subplots
        self.configure_subplots()
        
    def configure_subplots(self):
        """Configure all subplot properties"""
        # Row 1: Sensor Data
        self.ax_acc_comp.set_title('Gravity Compensated Acceleration', fontweight='bold')
        self.ax_acc_comp.set_ylabel('Acceleration (m/s²)')
        self.ax_acc_comp.grid(True, alpha=0.3)
        self.ax_acc_comp.legend(['X', 'Y', 'Z'], loc='upper right')
        
        self.ax_gyro.set_title('Gyroscope', fontweight='bold')
        self.ax_gyro.set_ylabel('Angular Velocity (rad/s)')
        self.ax_gyro.grid(True, alpha=0.3)
        self.ax_gyro.legend(['X', 'Y', 'Z'], loc='upper right')
        
        self.ax_euler.set_title('Euler Angles', fontweight='bold')
        self.ax_euler.set_ylabel('Angle (degrees)')
        self.ax_euler.grid(True, alpha=0.3)
        self.ax_euler.legend(['Roll', 'Pitch', 'Yaw'], loc='upper right')
        
        self.ax_temp_stat.set_title('Temperature & Motion Status', fontweight='bold')
        self.ax_temp_stat.set_ylabel('Temperature (°C)')
        self.ax_temp_stat.grid(True, alpha=0.3)
        
        # Row 2: Velocity Analysis
        self.ax_vel_body.set_title('Body Frame Velocity (Filtered)', fontweight='bold')
        self.ax_vel_body.set_ylabel('Velocity (m/s)')
        self.ax_vel_body.grid(True, alpha=0.3)
        self.ax_vel_body.legend(['Vx_body', 'Vy_body', 'Vz_body'], loc='upper right')
        
        self.ax_vel_world.set_title('World Frame Velocity ⭐', fontweight='bold', color='darkblue')
        self.ax_vel_world.set_ylabel('Velocity (m/s)')
        self.ax_vel_world.grid(True, alpha=0.3)
        self.ax_vel_world.legend(['Vx_world', 'Vy_world', 'Vz_world'], loc='upper right')
        
        self.ax_vel_comparison.set_title('Velocity Magnitude Comparison', fontweight='bold')
        self.ax_vel_comparison.set_ylabel('Speed (m/s)')
        self.ax_vel_comparison.grid(True, alpha=0.3)
        self.ax_vel_comparison.legend(['Body Speed', 'World Speed'], loc='upper right')
        
        self.ax_vel_magnitude.set_title('3D Velocity Components', fontweight='bold')
        self.ax_vel_magnitude.set_ylabel('Velocity (m/s)')
        self.ax_vel_magnitude.grid(True, alpha=0.3)
        
        # Row 3: Position Tracking
        self.ax_pos_world.set_title('World Position ⭐⭐', fontweight='bold', color='darkgreen')
        self.ax_pos_world.set_ylabel('Position (m)')
        self.ax_pos_world.grid(True, alpha=0.3)
        self.ax_pos_world.legend(['X_world', 'Y_world', 'Z_world'], loc='upper right')
        
        self.ax_pos_comparison.set_title('Position: World vs Body Frame', fontweight='bold')
        self.ax_pos_comparison.set_ylabel('Position (m)')
        self.ax_pos_comparison.grid(True, alpha=0.3)
        
        self.ax_trajectory_2d.set_title('2D Trajectory (Top View)', fontweight='bold')
        self.ax_trajectory_2d.set_xlabel('X Position (m)')
        self.ax_trajectory_2d.set_ylabel('Y Position (m)')
        self.ax_trajectory_2d.grid(True, alpha=0.3)
        self.ax_trajectory_2d.set_aspect('equal')
        
        self.ax_height_profile.set_title('Height Profile', fontweight='bold')
        self.ax_height_profile.set_ylabel('Z Position (m)')
        self.ax_height_profile.grid(True, alpha=0.3)
        
        # Row 4: 3D Visualization
        self.ax_3d_trajectory.set_title('3D Ball Trajectory ⭐⭐⭐', fontweight='bold', color='darkred')
        self.ax_3d_trajectory.set_xlabel('X Position (m)')
        self.ax_3d_trajectory.set_ylabel('Y Position (m)')
        self.ax_3d_trajectory.set_zlabel('Z Position (m)')
        
        self.ax_stats.set_title('Trajectory Statistics', fontweight='bold')
        self.ax_stats.axis('off')
        
        self.ax_motion_analysis.set_title('Motion Analysis', fontweight='bold')
        self.ax_motion_analysis.set_ylabel('Value')
        self.ax_motion_analysis.grid(True, alpha=0.3)
        
        # Row 5: Advanced Analysis
        self.ax_velocity_vectors.set_title('Velocity Vector Field', fontweight='bold')
        self.ax_velocity_vectors.set_xlabel('X Position (m)')
        self.ax_velocity_vectors.set_ylabel('Y Position (m)')
        self.ax_velocity_vectors.grid(True, alpha=0.3)
        
        self.ax_acceleration_world.set_title('World Frame Acceleration', fontweight='bold')
        self.ax_acceleration_world.set_ylabel('Acceleration (m/s²)')
        self.ax_acceleration_world.grid(True, alpha=0.3)
        
        self.ax_orientation_path.set_title('Orientation Evolution', fontweight='bold')
        self.ax_orientation_path.set_ylabel('Quaternion Components')
        self.ax_orientation_path.grid(True, alpha=0.3)
        
        self.ax_energy_analysis.set_title('Energy Analysis', fontweight='bold')
        self.ax_energy_analysis.set_ylabel('Energy (J/kg)')
        self.ax_energy_analysis.grid(True, alpha=0.3)
        
    def connect_serial(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            print(f"Connected to {self.serial_port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect to serial port: {e}")
            return False
            
    def read_serial_data(self):
        """Thread function to read serial data"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line and ',' in line:
                        # Skip header lines
                        if not line.startswith('timestamp') and not line.startswith('BMI088'):
                            try:
                                values = [float(x) for x in line.split(',')]
                                if len(values) == len(self.columns):
                                    self.data_queue.put(values)
                            except ValueError:
                                continue
            except Exception as e:
                print(f"Error reading serial data: {e}")
                break
                
    def load_csv_data(self):
        """Load data from CSV file"""
        try:
            self.df = pd.read_csv(self.csv_file)
            print(f"Loaded {len(self.df)} data points from {self.csv_file}")
            
            # Calculate derived values
            self.calculate_derived_values()
            
        except Exception as e:
            print(f"Error loading CSV file: {e}")
            
    def calculate_derived_values(self):
        """Calculate derived values for enhanced analysis"""
        if not self.df.empty:
            # Basic velocity magnitudes
            self.df['vel_mag_body'] = np.sqrt(self.df['vx_filt']**2 + self.df['vy_filt']**2 + self.df['vz_filt']**2)
            self.df['vel_mag_world'] = np.sqrt(self.df['vx_world']**2 + self.df['vy_world']**2 + self.df['vz_world']**2)
            
            # World frame accelerations (numerical differentiation)
            dt = 0.05  # 20Hz
            self.df['ax_world'] = np.gradient(self.df['vx_world'], dt)
            self.df['ay_world'] = np.gradient(self.df['vy_world'], dt)
            self.df['az_world'] = np.gradient(self.df['vz_world'], dt)
            
            # Energy calculations
            self.df['kinetic_energy'] = 0.5 * self.df['vel_mag_world']**2  # per unit mass
            self.df['potential_energy'] = 9.81 * self.df['pos_z_world']    # per unit mass (relative)
            self.df['total_energy'] = self.df['kinetic_energy'] + self.df['potential_energy']
            
            # Motion state analysis
            self.df['speed_world'] = self.df['vel_mag_world']
            self.df['acceleration_mag'] = np.sqrt(self.df['ax_world']**2 + self.df['ay_world']**2 + self.df['az_world']**2)
            
            # Trajectory curvature (simplified)
            self.df['direction_change'] = np.abs(np.gradient(np.arctan2(self.df['vy_world'], self.df['vx_world'])))
            
            # Normalize time for plotting
            if 'timestamp' in self.df.columns:
                self.df['time_sec'] = (self.df['timestamp'] - self.df['timestamp'].iloc[0]) / 1000.0
            else:
                self.df['time_sec'] = np.arange(len(self.df)) * dt
                
    def update_plot(self, frame):
        """Update plot for animation"""
        # Process new data from queue
        new_data = []
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                new_data.append(data)
            except queue.Empty:
                break
                
        if new_data:
            # Convert to DataFrame and append
            new_df = pd.DataFrame(new_data, columns=self.columns)
            self.df = pd.concat([self.df, new_df], ignore_index=True)
            
            # Keep only recent data
            if len(self.df) > self.max_points:
                self.df = self.df.tail(self.max_points).reset_index(drop=True)
                
            # Calculate derived values
            self.calculate_derived_values()
            
        if self.df.empty:
            return
            
        # Clear all axes
        axes_list = [
            self.ax_acc_comp, self.ax_gyro, self.ax_euler, self.ax_temp_stat,
            self.ax_vel_body, self.ax_vel_world, self.ax_vel_comparison, self.ax_vel_magnitude,
            self.ax_pos_world, self.ax_pos_comparison, self.ax_trajectory_2d, self.ax_height_profile,
            self.ax_3d_trajectory, self.ax_stats, self.ax_motion_analysis,
            self.ax_velocity_vectors, self.ax_acceleration_world, self.ax_orientation_path, self.ax_energy_analysis
        ]
        
        for ax in axes_list:
            if ax != self.ax_stats:  # Don't clear stats axis
                ax.clear()
            
        # Reconfigure after clearing
        self.configure_subplots()
        
        # Plot data
        self.plot_enhanced_data()
        
    def plot_enhanced_data(self):
        """Plot all enhanced data"""
        if self.df.empty:
            return
            
        time_data = self.df['time_sec'] if 'time_sec' in self.df.columns else range(len(self.df))
        
        # Row 1: Sensor Data
        # Gravity Compensated Acceleration
        self.ax_acc_comp.plot(time_data, self.df['g_acc_x'], 'r-', label='X', alpha=0.8)
        self.ax_acc_comp.plot(time_data, self.df['g_acc_y'], 'g-', label='Y', alpha=0.8)
        self.ax_acc_comp.plot(time_data, self.df['g_acc_z'], 'b-', label='Z', alpha=0.8)
        
        # Gyroscope
        self.ax_gyro.plot(time_data, self.df['raw_gx'], 'r-', label='X', alpha=0.8)
        self.ax_gyro.plot(time_data, self.df['raw_gy'], 'g-', label='Y', alpha=0.8)
        self.ax_gyro.plot(time_data, self.df['raw_gz'], 'b-', label='Z', alpha=0.8)
        
        # Euler Angles
        self.ax_euler.plot(time_data, self.df['roll'], 'r-', label='Roll', alpha=0.8)
        self.ax_euler.plot(time_data, self.df['pitch'], 'g-', label='Pitch', alpha=0.8)
        self.ax_euler.plot(time_data, self.df['yaw'], 'b-', label='Yaw', alpha=0.8)
        
        # Temperature and Stationary
        self.ax_temp_stat.plot(time_data, self.df['temp'], 'orange', label='Temperature', alpha=0.8)
        ax_twin = self.ax_temp_stat.twinx()
        ax_twin.plot(time_data, self.df['stationary'], 'purple', label='Stationary', alpha=0.8, linewidth=3)
        ax_twin.set_ylabel('Motion State')
        ax_twin.set_ylim(-0.1, 1.1)
        
        # Row 2: Velocity Analysis
        # Body Frame Velocity
        self.ax_vel_body.plot(time_data, self.df['vx_filt'], 'r-', label='Vx_body', alpha=0.8)
        self.ax_vel_body.plot(time_data, self.df['vy_filt'], 'g-', label='Vy_body', alpha=0.8)
        self.ax_vel_body.plot(time_data, self.df['vz_filt'], 'b-', label='Vz_body', alpha=0.8)
        
        # World Frame Velocity (HIGHLIGHTED)
        self.ax_vel_world.plot(time_data, self.df['vx_world'], 'r-', label='Vx_world', linewidth=2.5, alpha=0.9)
        self.ax_vel_world.plot(time_data, self.df['vy_world'], 'g-', label='Vy_world', linewidth=2.5, alpha=0.9)
        self.ax_vel_world.plot(time_data, self.df['vz_world'], 'b-', label='Vz_world', linewidth=2.5, alpha=0.9)
        
        # Velocity Magnitude Comparison
        if 'vel_mag_body' in self.df.columns:
            self.ax_vel_comparison.plot(time_data, self.df['vel_mag_body'], 'r--', label='Body Speed', alpha=0.7)
            self.ax_vel_comparison.plot(time_data, self.df['vel_mag_world'], 'b-', label='World Speed', linewidth=2, alpha=0.9)
        
        # 3D Velocity Components
        if 'vel_mag_world' in self.df.columns:
            self.ax_vel_magnitude.plot(time_data, np.abs(self.df['vx_world']), 'r-', label='|Vx|', alpha=0.8)
            self.ax_vel_magnitude.plot(time_data, np.abs(self.df['vy_world']), 'g-', label='|Vy|', alpha=0.8)
            self.ax_vel_magnitude.plot(time_data, np.abs(self.df['vz_world']), 'b-', label='|Vz|', alpha=0.8)
            self.ax_vel_magnitude.plot(time_data, self.df['vel_mag_world'], 'k-', label='Total', linewidth=2)
            self.ax_vel_magnitude.legend()
        
        # Row 3: Position Tracking
        # World Position (HIGHLIGHTED)
        self.ax_pos_world.plot(time_data, self.df['pos_x_world'], 'r-', label='X_world', linewidth=2.5, alpha=0.9)
        self.ax_pos_world.plot(time_data, self.df['pos_y_world'], 'g-', label='Y_world', linewidth=2.5, alpha=0.9)
        self.ax_pos_world.plot(time_data, self.df['pos_z_world'], 'b-', label='Z_world', linewidth=2.5, alpha=0.9)
        
        # Position Comparison
        self.ax_pos_comparison.plot(time_data, self.df['pos_x_world'], 'r-', label='X_world', linewidth=2)
        self.ax_pos_comparison.plot(time_data, self.df['pos_x_body'], 'r--', label='X_body', alpha=0.7)
        self.ax_pos_comparison.plot(time_data, self.df['pos_y_world'], 'g-', label='Y_world', linewidth=2)
        self.ax_pos_comparison.plot(time_data, self.df['pos_y_body'], 'g--', label='Y_body', alpha=0.7)
        self.ax_pos_comparison.legend()
        
        # 2D Trajectory (Top View)
        self.ax_trajectory_2d.plot(self.df['pos_x_world'], self.df['pos_y_world'], 'b-', alpha=0.6, linewidth=1.5)
        if len(self.df) > 0:
            self.ax_trajectory_2d.scatter(self.df['pos_x_world'].iloc[0], self.df['pos_y_world'].iloc[0], 
                                        c='green', s=100, marker='o', label='Start', zorder=5)
            self.ax_trajectory_2d.scatter(self.df['pos_x_world'].iloc[-1], self.df['pos_y_world'].iloc[-1], 
                                        c='red', s=100, marker='x', label='Current', zorder=5)
            self.ax_trajectory_2d.legend()
        
        # Height Profile
        self.ax_height_profile.plot(time_data, self.df['pos_z_world'], 'b-', linewidth=2, alpha=0.9)
        self.ax_height_profile.axhline(y=0, color='k', linestyle='--', alpha=0.5, label='Ground Level')
        self.ax_height_profile.fill_between(time_data, self.df['pos_z_world'], 0, alpha=0.3)
        self.ax_height_profile.legend()
        
        # Row 4: 3D Visualization & Analysis
        # 3D Trajectory (HIGHLIGHTED)
        if len(self.df) > 1:
            self.ax_3d_trajectory.plot(self.df['pos_x_world'], self.df['pos_y_world'], self.df['pos_z_world'], 
                                     'b-', alpha=0.7, linewidth=2)
            
            # Color-coded by speed
            if 'vel_mag_world' in self.df.columns:
                scatter = self.ax_3d_trajectory.scatter(self.df['pos_x_world'], self.df['pos_y_world'], self.df['pos_z_world'], 
                                                       c=self.df['vel_mag_world'], cmap='viridis', s=20, alpha=0.8)
                
            # Mark special points
            if len(self.df) > 0:
                self.ax_3d_trajectory.scatter(self.df['pos_x_world'].iloc[0], self.df['pos_y_world'].iloc[0], 
                                            self.df['pos_z_world'].iloc[0], c='green', s=150, marker='o', label='Start')
                self.ax_3d_trajectory.scatter(self.df['pos_x_world'].iloc[-1], self.df['pos_y_world'].iloc[-1], 
                                            self.df['pos_z_world'].iloc[-1], c='red', s=150, marker='x', label='Current')
            
            # Set equal aspect ratio
            max_range = max(
                self.df['pos_x_world'].max() - self.df['pos_x_world'].min(),
                self.df['pos_y_world'].max() - self.df['pos_y_world'].min(),
                self.df['pos_z_world'].max() - self.df['pos_z_world'].min()
            ) / 2.0
            
            if max_range > 0:
                mid_x = (self.df['pos_x_world'].max() + self.df['pos_x_world'].min()) * 0.5
                mid_y = (self.df['pos_y_world'].max() + self.df['pos_y_world'].min()) * 0.5
                mid_z = (self.df['pos_z_world'].max() + self.df['pos_z_world'].min()) * 0.5
                
                self.ax_3d_trajectory.set_xlim(mid_x - max_range, mid_x + max_range)
                self.ax_3d_trajectory.set_ylim(mid_y - max_range, mid_y + max_range)
                self.ax_3d_trajectory.set_zlim(mid_z - max_range, mid_z + max_range)
        
        # Statistics Display
        self.ax_stats.clear()
        self.ax_stats.axis('off')
        if len(self.df) > 0:
            current_stats = f"""
TRAJECTORY STATISTICS

Total Distance: {self.df['total_dist'].iloc[-1]:.3f} m
Max Height: {self.df['max_height'].iloc[-1]:.3f} m
Min Height: {self.df['min_height'].iloc[-1]:.3f} m

Current Position:
X: {self.df['pos_x_world'].iloc[-1]:.3f} m
Y: {self.df['pos_y_world'].iloc[-1]:.3f} m  
Z: {self.df['pos_z_world'].iloc[-1]:.3f} m

Current Speed: {self.df['vel_mag_world'].iloc[-1]:.3f} m/s

Motion State: {'STATIONARY' if self.df['stationary'].iloc[-1] else 'MOVING'}
Temperature: {self.df['temp'].iloc[-1]:.1f} °C
"""
            self.ax_stats.text(0.05, 0.95, current_stats, transform=self.ax_stats.transAxes, 
                             fontsize=10, verticalalignment='top', family='monospace',
                             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        # Motion Analysis
        if 'speed_world' in self.df.columns:
            self.ax_motion_analysis.plot(time_data, self.df['speed_world'], 'b-', label='Speed', linewidth=2)
            if 'acceleration_mag' in self.df.columns:
                ax_twin2 = self.ax_motion_analysis.twinx()
                ax_twin2.plot(time_data, self.df['acceleration_mag'], 'r-', label='Acceleration', alpha=0.7)
                ax_twin2.set_ylabel('Acceleration (m/s²)', color='red')
            self.ax_motion_analysis.legend(loc='upper left')
        
        # Row 5: Advanced Analysis
        # Velocity Vector Field
        if len(self.df) > 10:
            # Subsample for vector field
            step = max(1, len(self.df) // 20)
            x_pos = self.df['pos_x_world'][::step]
            y_pos = self.df['pos_y_world'][::step]
            vx = self.df['vx_world'][::step]
            vy = self.df['vy_world'][::step]
            
            self.ax_velocity_vectors.quiver(x_pos, y_pos, vx, vy, 
                                          self.df['vel_mag_world'][::step], 
                                          cmap='viridis', alpha=0.7, scale=5)
            self.ax_velocity_vectors.plot(self.df['pos_x_world'], self.df['pos_y_world'], 'k-', alpha=0.3)
        
        # World Frame Acceleration
        if 'ax_world' in self.df.columns:
            self.ax_acceleration_world.plot(time_data, self.df['ax_world'], 'r-', label='Ax_world', alpha=0.8)
            self.ax_acceleration_world.plot(time_data, self.df['ay_world'], 'g-', label='Ay_world', alpha=0.8)
            self.ax_acceleration_world.plot(time_data, self.df['az_world'], 'b-', label='Az_world', alpha=0.8)
            self.ax_acceleration_world.legend()
        
        # Orientation Evolution
        self.ax_orientation_path.plot(time_data, self.df['q0'], 'r-', label='q0', alpha=0.8)
        self.ax_orientation_path.plot(time_data, self.df['q1'], 'g-', label='q1', alpha=0.8)
        self.ax_orientation_path.plot(time_data, self.df['q2'], 'b-', label='q2', alpha=0.8)
        self.ax_orientation_path.plot(time_data, self.df['q3'], 'm-', label='q3', alpha=0.8)
        self.ax_orientation_path.legend()
        
        # Energy Analysis
        if 'kinetic_energy' in self.df.columns:
            self.ax_energy_analysis.plot(time_data, self.df['kinetic_energy'], 'r-', label='Kinetic', linewidth=2)
            self.ax_energy_analysis.plot(time_data, self.df['potential_energy'], 'b-', label='Potential', linewidth=2)
            self.ax_energy_analysis.plot(time_data, self.df['total_energy'], 'k-', label='Total', linewidth=2, alpha=0.8)
            self.ax_energy_analysis.legend()
            
        # Update legends for all plots
        for ax in [self.ax_acc_comp, self.ax_gyro, self.ax_euler, self.ax_vel_body, 
                  self.ax_vel_world, self.ax_vel_comparison, self.ax_pos_world]:
            ax.legend(loc='upper right', fontsize=8)
            
    def start_visualization(self):
        """Start the visualization"""
        if self.real_time and self.serial_port:
            if self.connect_serial():
                self.running = True
                # Start serial reading thread
                serial_thread = threading.Thread(target=self.read_serial_data)
                serial_thread.daemon = True
                serial_thread.start()
                
                # Start animation
                self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
            else:
                print("Failed to connect to serial port")
                return
        elif self.csv_file:
            self.load_csv_data()
            self.plot_enhanced_data()
        else:
            print("No data source specified")
            return
            
        plt.tight_layout()
        plt.show()
        
    def stop_visualization(self):
        """Stop the visualization"""
        self.running = False
        if self.ser:
            self.ser.close()
            
    def save_data(self, filename):
        """Save current data to CSV"""
        if not self.df.empty:
            self.df.to_csv(filename, index=False)
            print(f"Data saved to {filename}")
            
    def export_trajectory(self, filename):
        """Export trajectory data for external analysis"""
        if not self.df.empty:
            trajectory_data = self.df[['time_sec', 'pos_x_world', 'pos_y_world', 'pos_z_world', 
                                     'vx_world', 'vy_world', 'vz_world', 'vel_mag_world', 
                                     'total_dist', 'stationary']].copy()
            trajectory_data.to_csv(filename, index=False)
            print(f"Trajectory data exported to {filename}")
            
    def generate_report(self):
        """Generate a summary report of the ball's motion"""
        if self.df.empty:
            print("No data available for report generation")
            return
            
        report = f"""
THYNKBALL MOTION ANALYSIS REPORT
================================
Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

TRAJECTORY SUMMARY:
- Total recording time: {self.df['time_sec'].iloc[-1]:.2f} seconds
- Total distance traveled: {self.df['total_dist'].iloc[-1]:.3f} meters
- Maximum height reached: {self.df['max_height'].iloc[-1]:.3f} meters
- Minimum height reached: {self.df['min_height'].iloc[-1]:.3f} meters

VELOCITY STATISTICS:
- Maximum speed: {self.df['vel_mag_world'].max():.3f} m/s
- Average speed: {self.df['vel_mag_world'].mean():.3f} m/s
- Time spent stationary: {(self.df['stationary'].sum() / len(self.df)) * 100:.1f}%

POSITION RANGE:
- X-axis range: {self.df['pos_x_world'].min():.3f} to {self.df['pos_x_world'].max():.3f} meters
- Y-axis range: {self.df['pos_y_world'].min():.3f} to {self.df['pos_y_world'].max():.3f} meters
- Z-axis range: {self.df['pos_z_world'].min():.3f} to {self.df['pos_z_world'].max():.3f} meters

SENSOR PERFORMANCE:
- Temperature range: {self.df['temp'].min():.1f}°C to {self.df['temp'].max():.1f}°C
- Temperature stability: {self.df['temp'].std():.2f}°C std dev

ENERGY ANALYSIS:
"""
        if 'kinetic_energy' in self.df.columns:
            report += f"""- Maximum kinetic energy: {self.df['kinetic_energy'].max():.3f} J/kg
- Maximum potential energy: {self.df['potential_energy'].max():.3f} J/kg
- Energy efficiency: {((self.df['total_energy'].iloc[-1] / self.df['total_energy'].iloc[0]) * 100):.1f}% retained
"""
        
        print(report)
        return report

def main():
    parser = argparse.ArgumentParser(description='Enhanced Thynkball IMU & Position Visualizer')
    parser.add_argument('--port', '-p', type=str, help='Serial port (e.g., COM3 or /dev/ttyUSB0)')
    parser.add_argument('--baud', '-b', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--csv', '-c', type=str, help='CSV file to load and visualize')
    parser.add_argument('--save', '-s', type=str, help='Save data to CSV file')
    parser.add_argument('--export', '-e', type=str, help='Export trajectory data to CSV')
    parser.add_argument('--report', '-r', action='store_true', help='Generate motion analysis report')
    
    args = parser.parse_args()
    
    # Determine mode
    if args.csv:
        # Static visualization from CSV
        visualizer = EnhancedThynkballVisualizer(csv_file=args.csv, real_time=False)
    elif args.port:
        # Real-time visualization from serial
        visualizer = EnhancedThynkballVisualizer(serial_port=args.port, baudrate=args.baud, real_time=True)
    else:
        print("Please specify either --port for real-time data or --csv for file visualization")
        print("Example: python enhanced_thynkball_visualizer.py --port COM3")
        print("Example: python enhanced_thynkball_visualizer.py --csv data.csv --report")
        return
    
    try:
        visualizer.start_visualization()
    except KeyboardInterrupt:
        print("\nStopping visualization...")
        visualizer.stop_visualization()
        
        # Generate report if requested
        if args.report:
            visualizer.generate_report()
        
        # Save data if requested
        if args.save and not visualizer.df.empty:
            visualizer.save_data(args.save)
            
        # Export trajectory if requested
        if args.export and not visualizer.df.empty:
            visualizer.export_trajectory(args.export)

if __name__ == "__main__":
    main()