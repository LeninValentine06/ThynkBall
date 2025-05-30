#!/usr/bin/env python3
"""
BMI088 IMU Data Logger
Captures serial data from Arduino/ESP32 and saves to CSV with real-time monitoring
"""

import serial
import csv
import time
import os
import argparse
from datetime import datetime
import signal
import sys

class BMI088DataLogger:
    def __init__(self, port='COM3', baudrate=115200, output_dir='imu_data'):
        self.port = port
        self.baudrate = baudrate
        self.output_dir = output_dir
        self.ser = None
        self.csv_writer = None
        self.csv_file = None
        self.sample_count = 0
        self.start_time = None
        self.running = True
        
        # CSV column headers (matching Arduino output)
        self.headers = [
            'timestamp_local',  # Added local timestamp
            'raw_acc_x', 'raw_acc_y', 'raw_acc_z',
            'g_acc_x', 'g_acc_y', 'g_acc_z',
            'raw_gx', 'raw_gy', 'raw_gz',
            'filt_gx', 'filt_gy', 'filt_gz',
            'temp', 'roll', 'pitch', 'yaw',
            'q0', 'q1', 'q2', 'q3',
            'vx', 'vy', 'vz',
            'vx_filt', 'vy_filt', 'vz_filt',
            'px', 'py', 'pz',
            'stationary'
        ]
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print(f"\n\nReceived interrupt signal. Saving data...")
        self.running = False
        self.cleanup()
        sys.exit(0)
    
    def find_arduino_port(self):
        """Try to automatically find Arduino/ESP32 port"""
        import serial.tools.list_ports
        
        print("Scanning for Arduino/ESP32 devices...")
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            print(f"Found device: {port.device} - {port.description}")
            if any(keyword in port.description.lower() for keyword in 
                   ['arduino', 'esp32', 'cp210', 'ch340', 'ftdi', 'usb-serial']):
                print(f"Potential Arduino/ESP32 found: {port.device}")
                return port.device
        
        return None
    
    def connect_serial(self):
        """Establish serial connection"""
        if self.port == 'auto':
            self.port = self.find_arduino_port()
            if not self.port:
                print("No Arduino/ESP32 device found automatically.")
                return False
        
        try:
            print(f"Connecting to {self.port} at {self.baudrate} baud...")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Allow Arduino to reset
            
            # Clear any initial data
            self.ser.flushInput()
            
            print("Serial connection established!")
            return True
            
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
    
    def setup_csv_file(self, filename=None):
        """Setup CSV file for writing"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"bmi088_data_{timestamp}.csv"
        
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            self.csv_file = open(filepath, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(self.headers)
            self.csv_file.flush()
            
            print(f"CSV file created: {filepath}")
            return True
            
        except Exception as e:
            print(f"Failed to create CSV file: {e}")
            return False
    
    def parse_arduino_data(self, line):
        """Parse data line from Arduino"""
        try:
            # Remove whitespace and split by comma
            data = line.strip().split(',')
            
            # Skip header lines or incomplete data
            if len(data) != len(self.headers) - 1:  # -1 because we add local timestamp
                return None
            
            # Convert to appropriate data types
            parsed_data = []
            for i, value in enumerate(data):
                try:
                    # Most values are floats, except temp (int) and stationary (int)
                    if i == 12:  # temp column
                        parsed_data.append(int(float(value)))
                    elif i == len(data) - 1:  # stationary column
                        parsed_data.append(int(float(value)))
                    else:
                        parsed_data.append(float(value))
                except ValueError:
                    return None
            
            return parsed_data
            
        except Exception as e:
            return None
    
    def log_data(self):
        """Main data logging loop"""
        self.start_time = time.time()
        last_progress_time = time.time()
        
        print("\n" + "="*60)
        print("BMI088 Data Logging Started")
        print("Press Ctrl+C to stop and save data")
        print("="*60)
        
        while self.running:
            try:
                # Read line from serial
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore')
                    
                    # Skip empty lines or debug messages
                    if not line.strip() or "BMI088" in line or "bias" in line:
                        continue
                    
                    # Parse the data
                    parsed_data = self.parse_arduino_data(line)
                    
                    if parsed_data:
                        # Add local timestamp as first column
                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        row_data = [timestamp] + parsed_data
                        
                        # Write to CSV
                        self.csv_writer.writerow(row_data)
                        self.csv_file.flush()  # Ensure data is written immediately
                        
                        self.sample_count += 1
                        
                        # Print progress every 5 seconds
                        current_time = time.time()
                        if current_time - last_progress_time >= 5.0:
                            elapsed = current_time - self.start_time
                            rate = self.sample_count / elapsed if elapsed > 0 else 0
                            
                            print(f"Samples: {self.sample_count:6d} | "
                                  f"Elapsed: {elapsed:6.1f}s | "
                                  f"Rate: {rate:5.1f} Hz | "
                                  f"Stationary: {'Yes' if parsed_data[-1] == 1 else 'No ':3s}")
                            
                            last_progress_time = current_time
                
                else:
                    time.sleep(0.001)  # Small delay to prevent busy waiting
                    
            except Exception as e:
                print(f"Error during logging: {e}")
                continue
    
    def cleanup(self):
        """Clean up resources"""
        if self.csv_file:
            self.csv_file.close()
            print(f"CSV file saved with {self.sample_count} samples")
        
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")
    
    def run(self, filename=None):
        """Main execution function"""
        print("BMI088 IMU Data Logger Starting...")
        
        # Connect to serial port
        if not self.connect_serial():
            return False
        
        # Setup CSV file
        if not self.setup_csv_file(filename):
            self.cleanup()
            return False
        
        try:
            # Start logging
            self.log_data()
            
        except Exception as e:
            print(f"Unexpected error: {e}")
        
        finally:
            self.cleanup()
        
        return True


def main():
    parser = argparse.ArgumentParser(description='BMI088 IMU Data Logger')
    parser.add_argument('-p', '--port', default='auto',
                        help='Serial port (default: auto-detect)')
    parser.add_argument('-b', '--baud', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('-o', '--output', default='imu_data',
                        help='Output directory (default: imu_data)')
    parser.add_argument('-f', '--filename', default=None,
                        help='Custom CSV filename (optional)')
    
    args = parser.parse_args()
    
    # Create logger instance
    logger = BMI088DataLogger(
        port=args.port,
        baudrate=args.baud,
        output_dir=args.output
    )
    
    # Run the logger
    success = logger.run(args.filename)
    
    if success:
        print("\nData logging completed successfully!")
    else:
        print("\nData logging failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()