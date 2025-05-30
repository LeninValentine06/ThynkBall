#!/usr/bin/env python3
"""
BMI088 IMU Real-Time Dashboard
Live visualization of sensor data with multiple plots and metrics
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import numpy as np
import serial
import threading
import queue
import time
from collections import deque
import json
import os
from datetime import datetime

class BMI088Dashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("BMI088 IMU Real-Time Dashboard")
        self.root.geometry("1400x900")
        
        # Serial connection
        self.ser = None
        self.connected = False
        self.data_queue = queue.Queue()
        self.serial_thread = None
        
        # Data storage (keep last 1000 points for smooth plotting)
        self.max_points = 1000
        self.time_data = deque(maxlen=self.max_points)
        
        # Accelerometer data
        self.acc_x = deque(maxlen=self.max_points)
        self.acc_y = deque(maxlen=self.max_points)
        self.acc_z = deque(maxlen=self.max_points)
        
        # Gravity-compensated acceleration
        self.g_acc_x = deque(maxlen=self.max_points)
        self.g_acc_y = deque(maxlen=self.max_points)
        self.g_acc_z = deque(maxlen=self.max_points)
        
        # Gyroscope data
        self.gyro_x = deque(maxlen=self.max_points)
        self.gyro_y = deque(maxlen=self.max_points)
        self.gyro_z = deque(maxlen=self.max_points)
        
        # Orientation data
        self.roll = deque(maxlen=self.max_points)
        self.pitch = deque(maxlen=self.max_points)
        self.yaw = deque(maxlen=self.max_points)
        
        # Velocity and position
        self.vel_x = deque(maxlen=self.max_points)
        self.vel_y = deque(maxlen=self.max_points)
        self.vel_z = deque(maxlen=self.max_points)
        self.pos_x = deque(maxlen=self.max_points)
        self.pos_y = deque(maxlen=self.max_points)
        self.pos_z = deque(maxlen=self.max_points)
        
        # Status data
        self.temperature = deque(maxlen=self.max_points)
        self.stationary = deque(maxlen=self.max_points)
        
        # Statistics
        self.sample_count = 0
        self.start_time = None
        self.last_update = time.time()
        
        # Setup GUI
        self.setup_gui()
        self.setup_plots()
        
        # Start animation
        self.ani = FuncAnimation(self.fig, self.update_plots, interval=50, blit=False)
        
    def setup_gui(self):
        """Setup the GUI layout"""
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Control panel (top)
        control_frame = ttk.LabelFrame(main_frame, text="Control Panel", padding=10)
        control_frame.pack(fill=tk.X, pady=(0, 5))
        
        # Serial connection controls
        serial_frame = ttk.Frame(control_frame)
        serial_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        ttk.Label(serial_frame, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value="COM5")
        port_entry = ttk.Entry(serial_frame, textvariable=self.port_var, width=10)
        port_entry.pack(side=tk.LEFT, padx=(5, 10))
        
        self.connect_btn = ttk.Button(serial_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        # Status indicators
        status_frame = ttk.Frame(control_frame)
        status_frame.pack(side=tk.RIGHT)
        
        self.status_label = ttk.Label(status_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=10)
        
        self.rate_label = ttk.Label(status_frame, text="Rate: 0.0 Hz")
        self.rate_label.pack(side=tk.LEFT, padx=10)
        
        self.samples_label = ttk.Label(status_frame, text="Samples: 0")
        self.samples_label.pack(side=tk.LEFT, padx=10)
        
        # Action buttons
        action_frame = ttk.Frame(control_frame)
        action_frame.pack(side=tk.RIGHT, padx=(20, 0))
        
        ttk.Button(action_frame, text="Save Data", command=self.save_data).pack(side=tk.LEFT, padx=5)
        ttk.Button(action_frame, text="Clear Data", command=self.clear_data).pack(side=tk.LEFT, padx=5)
        
        # Metrics panel (left side)
        metrics_frame = ttk.LabelFrame(main_frame, text="Live Metrics", padding=10)
        metrics_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
        
        self.setup_metrics_panel(metrics_frame)
        
        # Plots panel (right side)
        plots_frame = ttk.Frame(main_frame)
        plots_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Create matplotlib figure
        self.fig, self.axes = plt.subplots(3, 2, figsize=(12, 8))
        self.fig.tight_layout(pad=3.0)
        
        # Embed matplotlib in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, plots_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def setup_metrics_panel(self, parent):
        """Setup the live metrics display"""
        self.metric_vars = {}
        
        metrics = [
            ("Acceleration (m/s²)", ["X", "Y", "Z"]),
            ("Linear Accel (m/s²)", ["X", "Y", "Z"]),
            ("Gyroscope (°/s)", ["X", "Y", "Z"]),
            ("Orientation (°)", ["Roll", "Pitch", "Yaw"]),
            ("Velocity (m/s)", ["X", "Y", "Z"]),
            ("Position (m)", ["X", "Y", "Z"]),
            ("Status", ["Temp (°C)", "Stationary", ""])
        ]
        
        for section, items in metrics:
            # Section header
            section_frame = ttk.LabelFrame(parent, text=section, padding=5)
            section_frame.pack(fill=tk.X, pady=5)
            
            for item in items:
                if item:  # Skip empty items
                    frame = ttk.Frame(section_frame)
                    frame.pack(fill=tk.X, pady=1)
                    
                    ttk.Label(frame, text=f"{item}:", width=10).pack(side=tk.LEFT)
                    var = tk.StringVar(value="0.000")
                    self.metric_vars[f"{section}_{item}"] = var
                    ttk.Label(frame, textvariable=var, width=10, foreground="blue").pack(side=tk.LEFT)
        
    def setup_plots(self):
        """Setup the plot configurations"""
        # Plot titles and configurations
        plot_configs = [
            ("Raw Accelerometer", "Time (s)", "Acceleration (m/s²)"),
            ("Linear Acceleration", "Time (s)", "Acceleration (m/s²)"),
            ("Gyroscope", "Time (s)", "Angular Velocity (°/s)"),
            ("Orientation", "Time (s)", "Angle (°)"),
            ("Velocity", "Time (s)", "Velocity (m/s)"),
            ("Position", "Time (s)", "Position (m)")
        ]
        
        # Setup each subplot
        for i, (title, xlabel, ylabel) in enumerate(plot_configs):
            row, col = i // 2, i % 2
            ax = self.axes[row, col]
            ax.set_title(title, fontsize=10)
            ax.set_xlabel(xlabel, fontsize=8)
            ax.set_ylabel(ylabel, fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
        
        # Set dark background for better visibility
        plt.style.use('default')
        self.fig.patch.set_facecolor('white')
        
    def toggle_connection(self):
        """Toggle serial connection"""
        if self.connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        """Connect to serial port"""
        try:
            port = self.port_var.get()
            self.ser = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)  # Allow Arduino to reset
            self.ser.flushInput()
            
            self.connected = True
            self.start_time = time.time()
            self.sample_count = 0
            
            # Start serial reading thread
            self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.serial_thread.start()
            
            # Update GUI
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Status: Connected", foreground="green")
            
            messagebox.showinfo("Success", f"Connected to {port}")
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.connected = False
        
        if self.ser and self.ser.is_open:
            self.ser.close()
        
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.rate_label.config(text="Rate: 0.0 Hz")
    
    def read_serial(self):
        """Read data from serial port in separate thread"""
        while self.connected:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Skip header lines or debug messages
                    if not line or "BMI088" in line or "bias" in line:
                        continue
                    
                    # Parse data
                    data = self.parse_data_line(line)
                    if data:
                        self.data_queue.put(data)
                
                time.sleep(0.001)  # Small delay
                
            except Exception as e:
                if self.connected:  # Only show error if we should be connected
                    print(f"Serial read error: {e}")
                break
    
    def parse_data_line(self, line):
        """Parse a line of data from Arduino"""
        try:
            values = line.split(',')
            if len(values) != 30:  # Expected number of values
                return None
            
            # Convert to floats
            data = [float(v) for v in values]
            return data
            
        except (ValueError, IndexError):
            return None
    
    def update_plots(self, frame):
        """Update plots with new data"""
        # Process any new data from queue
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                self.add_data_point(data)
            except queue.Empty:
                break
        
        # Update rate calculation
        current_time = time.time()
        if self.start_time:
            elapsed = current_time - self.start_time
            rate = self.sample_count / elapsed if elapsed > 0 else 0
            self.rate_label.config(text=f"Rate: {rate:.1f} Hz")
            self.samples_label.config(text=f"Samples: {self.sample_count}")
        
        # Clear all axes
        for ax in self.axes.flat:
            ax.clear()
        
        if len(self.time_data) < 2:
            return
        
        # Convert time to relative seconds
        time_array = np.array(self.time_data)
        time_rel = time_array - time_array[0] if len(time_array) > 0 else time_array
        
        # Plot 1: Raw Accelerometer
        ax = self.axes[0, 0]
        ax.plot(time_rel, self.acc_x, 'r-', label='X', linewidth=1)
        ax.plot(time_rel, self.acc_y, 'g-', label='Y', linewidth=1)
        ax.plot(time_rel, self.acc_z, 'b-', label='Z', linewidth=1)
        ax.set_title("Raw Accelerometer", fontsize=10)
        ax.set_ylabel("Acceleration (m/s²)", fontsize=8)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # Plot 2: Linear Acceleration (gravity compensated)
        ax = self.axes[0, 1]
        ax.plot(time_rel, self.g_acc_x, 'r-', label='X', linewidth=1)
        ax.plot(time_rel, self.g_acc_y, 'g-', label='Y', linewidth=1)
        ax.plot(time_rel, self.g_acc_z, 'b-', label='Z', linewidth=1)
        ax.set_title("Linear Acceleration", fontsize=10)
        ax.set_ylabel("Acceleration (m/s²)", fontsize=8)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # Plot 3: Gyroscope (convert to degrees/s)
        ax = self.axes[1, 0]
        gyro_x_deg = np.array(self.gyro_x) * 180.0 / np.pi
        gyro_y_deg = np.array(self.gyro_y) * 180.0 / np.pi
        gyro_z_deg = np.array(self.gyro_z) * 180.0 / np.pi
        ax.plot(time_rel, gyro_x_deg, 'r-', label='X', linewidth=1)
        ax.plot(time_rel, gyro_y_deg, 'g-', label='Y', linewidth=1)
        ax.plot(time_rel, gyro_z_deg, 'b-', label='Z', linewidth=1)
        ax.set_title("Gyroscope", fontsize=10)
        ax.set_ylabel("Angular Velocity (°/s)", fontsize=8)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # Plot 4: Orientation
        ax = self.axes[1, 1]
        ax.plot(time_rel, self.roll, 'r-', label='Roll', linewidth=1)
        ax.plot(time_rel, self.pitch, 'g-', label='Pitch', linewidth=1)
        ax.plot(time_rel, self.yaw, 'b-', label='Yaw', linewidth=1)
        ax.set_title("Orientation", fontsize=10)
        ax.set_ylabel("Angle (°)", fontsize=8)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # Plot 5: Velocity
        ax = self.axes[2, 0]
        ax.plot(time_rel, self.vel_x, 'r-', label='X', linewidth=1)
        ax.plot(time_rel, self.vel_y, 'g-', label='Y', linewidth=1)
        ax.plot(time_rel, self.vel_z, 'b-', label='Z', linewidth=1)
        ax.set_title("Velocity", fontsize=10)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel("Velocity (m/s)", fontsize=8)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # Plot 6: Position
        ax = self.axes[2, 1]
        ax.plot(time_rel, self.pos_x, 'r-', label='X', linewidth=1)
        ax.plot(time_rel, self.pos_y, 'g-', label='Y', linewidth=1)
        ax.plot(time_rel, self.pos_z, 'b-', label='Z', linewidth=1)
        ax.set_title("Position", fontsize=10)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel("Position (m)", fontsize=8)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # Adjust layout and redraw
        self.fig.tight_layout(pad=2.0)
        self.canvas.draw_idle()
    
    def add_data_point(self, data):
        """Add a new data point to all deques"""
        current_time = time.time()
        self.time_data.append(current_time)
        
        # Raw accelerometer (indices 0-2)
        self.acc_x.append(data[0])
        self.acc_y.append(data[1])
        self.acc_z.append(data[2])
        
        # Gravity-compensated acceleration (indices 3-5)
        self.g_acc_x.append(data[3])
        self.g_acc_y.append(data[4])
        self.g_acc_z.append(data[5])
        
        # Raw gyroscope (indices 6-8)
        self.gyro_x.append(data[6])
        self.gyro_y.append(data[7])
        self.gyro_z.append(data[8])
        
        # Skip filtered gyro (indices 9-11)
        # Temperature (index 12)
        temp = data[12]
        self.temperature.append(temp)
        
        # Orientation (indices 13-15)
        self.roll.append(data[13])
        self.pitch.append(data[14])
        self.yaw.append(data[15])
        
        # Skip quaternions (indices 16-19)
        
        # Velocity (indices 20-22) and filtered velocity (indices 23-25)
        self.vel_x.append(data[23])  # Use filtered velocity
        self.vel_y.append(data[24])
        self.vel_z.append(data[25])
        
        # Position (indices 26-28)
        self.pos_x.append(data[26])
        self.pos_y.append(data[27])
        self.pos_z.append(data[28])
        
        # Stationary flag (index 29)
        self.stationary.append(data[29])
        
        self.sample_count += 1
        
        # Update live metrics
        self.update_metrics(data)
    
    def update_metrics(self, data):
        """Update the live metrics display"""
        try:
            # Accelerometer
            self.metric_vars["Acceleration (m/s²)_X"].set(f"{data[0]:.3f}")
            self.metric_vars["Acceleration (m/s²)_Y"].set(f"{data[1]:.3f}")
            self.metric_vars["Acceleration (m/s²)_Z"].set(f"{data[2]:.3f}")
            
            # Linear acceleration
            self.metric_vars["Linear Accel (m/s²)_X"].set(f"{data[3]:.3f}")
            self.metric_vars["Linear Accel (m/s²)_Y"].set(f"{data[4]:.3f}")
            self.metric_vars["Linear Accel (m/s²)_Z"].set(f"{data[5]:.3f}")
            
            # Gyroscope (convert to degrees/s)
            self.metric_vars["Gyroscope (°/s)_X"].set(f"{data[6] * 180.0 / np.pi:.2f}")
            self.metric_vars["Gyroscope (°/s)_Y"].set(f"{data[7] * 180.0 / np.pi:.2f}")
            self.metric_vars["Gyroscope (°/s)_Z"].set(f"{data[8] * 180.0 / np.pi:.2f}")
            
            # Orientation
            self.metric_vars["Orientation (°)_Roll"].set(f"{data[13]:.1f}")
            self.metric_vars["Orientation (°)_Pitch"].set(f"{data[14]:.1f}")
            self.metric_vars["Orientation (°)_Yaw"].set(f"{data[15]:.1f}")
            
            # Velocity (filtered)
            self.metric_vars["Velocity (m/s)_X"].set(f"{data[23]:.3f}")
            self.metric_vars["Velocity (m/s)_Y"].set(f"{data[24]:.3f}")
            self.metric_vars["Velocity (m/s)_Z"].set(f"{data[25]:.3f}")
            
            # Position
            self.metric_vars["Position (m)_X"].set(f"{data[26]:.3f}")
            self.metric_vars["Position (m)_Y"].set(f"{data[27]:.3f}")
            self.metric_vars["Position (m)_Z"].set(f"{data[28]:.3f}")
            
            # Status
            self.metric_vars["Status_Temp (°C)"].set(f"{data[12]:.1f}")
            self.metric_vars["Status_Stationary"].set("Yes" if data[29] == 1 else "No")
            
        except KeyError as e:
            print(f"Metric update error: {e}")
    
    def save_data(self):
        """Save current data to CSV file"""
        if self.sample_count == 0:
            messagebox.showwarning("No Data", "No data to save!")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialname=f"bmi088_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        
        if filename:
            try:
                # Save data with headers
                headers = [
                    'timestamp', 'raw_acc_x', 'raw_acc_y', 'raw_acc_z',
                    'g_acc_x', 'g_acc_y', 'g_acc_z',
                    'gyro_x', 'gyro_y', 'gyro_z',
                    'roll', 'pitch', 'yaw',
                    'vel_x', 'vel_y', 'vel_z',
                    'pos_x', 'pos_y', 'pos_z',
                    'temperature', 'stationary'
                ]
                
                with open(filename, 'w') as f:
                    f.write(','.join(headers) + '\n')
                    
                    for i in range(len(self.time_data)):
                        row = [
                            self.time_data[i],
                            self.acc_x[i], self.acc_y[i], self.acc_z[i],
                            self.g_acc_x[i], self.g_acc_y[i], self.g_acc_z[i],
                            self.gyro_x[i], self.gyro_y[i], self.gyro_z[i],
                            self.roll[i], self.pitch[i], self.yaw[i],
                            self.vel_x[i], self.vel_y[i], self.vel_z[i],
                            self.pos_x[i], self.pos_y[i], self.pos_z[i],
                            self.temperature[i], self.stationary[i]
                        ]
                        f.write(','.join(map(str, row)) + '\n')
                
                messagebox.showinfo("Success", f"Data saved to {filename}")
                
            except Exception as e:
                messagebox.showerror("Save Error", f"Failed to save data: {str(e)}")
    
    def clear_data(self):
        """Clear all stored data"""
        if messagebox.askyesno("Clear Data", "Are you sure you want to clear all data?"):
            # Clear all deques
            for attr_name in dir(self):
                attr = getattr(self, attr_name)
                if isinstance(attr, deque):
                    attr.clear()
            
            self.sample_count = 0
            self.start_time = time.time() if self.connected else None
            
            # Reset metric displays
            for var in self.metric_vars.values():
                var.set("0.000")
    
    def on_closing(self):
        """Handle window closing"""
        self.disconnect()
        self.root.quit()


def main():
    root = tk.Tk()
    dashboard = BMI088Dashboard(root)
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", dashboard.on_closing)
    
    # Start the GUI
    root.mainloop()


if __name__ == "__main__":
    main()