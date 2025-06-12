#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math
import numpy as np
from collections import deque
import csv
import os
from datetime import datetime

class AdvancedSerialImuPublisher(Node):
    def __init__(self):
        super().__init__('advanced_serial_imu_publisher')
        
        # Create publisher for IMU data
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        
        # Debug flag
        self.debug_filtering = False
        
        # CSV logging parameters
        self.enable_csv_logging = True  # Set to True to enable CSV logging
        self.csv_file = None
        self.csv_writer = None
        self.csv_filename = None
        
        # Choose filtering method (uncomment one):
        self.filter_method = "complementary"  # Best for terrain response
        # self.filter_method = "adaptive_alpha"  # Good balance
        # self.filter_method = "median_ma"      # Simple and effective
        # self.filter_method = "kalman"         # Most sophisticated
        
        # Initialize CSV logging if enabled
        if self.enable_csv_logging:
            self.setup_csv_logging()
        
        # Initialize filters based on chosen method
        self.setup_filters()
        
        # Initialize serial connection
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info('Serial connection established on /dev/ttyUSB0')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return
            
        # Timer to read serial data at 10 Hz
        self.timer = self.create_timer(0.1, self.read_serial_data)
        self.dt = 0.1  # Sample time
        
        # Initialize variables for orientation estimation
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.get_logger().info(f'Advanced IMU Publisher started - Using {self.filter_method} filtering')
        if self.enable_csv_logging:
            self.get_logger().info(f'CSV logging enabled - File: {self.csv_filename}')

    def setup_csv_logging(self):
        """Initialize CSV file for logging IMU data"""
        try:
            # Create logs directory if it doesn't exist
            log_dir = "imu_logs"
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_filename = os.path.join(log_dir, f"imu_data_{timestamp}.csv")
            
            # Open CSV file and create writer
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write header row
            header = [
                'timestamp_ms',
                'raw_ax', 'raw_ay', 'raw_az',
                'raw_gx', 'raw_gy', 'raw_gz',
                'filtered_ax', 'filtered_ay', 'filtered_az',
                'filtered_gx', 'filtered_gy', 'filtered_gz',
                'roll_deg', 'pitch_deg', 'yaw_deg',
                'qx', 'qy', 'qz', 'qw',
                'filter_method'
            ]
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            
            self.get_logger().info(f'CSV logging initialized: {self.csv_filename}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup CSV logging: {e}')
            self.enable_csv_logging = False

    def save_to_csv(self, timestamp_ms, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz,
                   filtered_ax, filtered_ay, filtered_az, filtered_gx, filtered_gy, filtered_gz,
                   roll, pitch, yaw, qx, qy, qz, qw):
        """Save IMU data to CSV file"""
        if not self.enable_csv_logging or not self.csv_writer:
            return
        
        try:
            # Convert angles to degrees for easier reading
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            
            # Write data row
            row = [
                timestamp_ms,
                raw_ax, raw_ay, raw_az,
                raw_gx, raw_gy, raw_gz,
                filtered_ax, filtered_ay, filtered_az,
                filtered_gx, filtered_gy, filtered_gz,
                roll_deg, pitch_deg, yaw_deg,
                qx, qy, qz, qw,
                self.filter_method
            ]
            
            self.csv_writer.writerow(row)
            
            # Flush periodically to ensure data is written
            if timestamp_ms % 1000 < 100:  # Flush every ~1 second
                self.csv_file.flush()
                
        except Exception as e:
            self.get_logger().error(f'Error writing to CSV: {e}')

    def setup_filters(self):
        """Initialize filters based on chosen method"""
        
        if self.filter_method == "complementary":
            # Complementary Filter - Excellent for terrain response
            self.comp_alpha = 0.98  # Trust gyro for short-term, accel for long-term
            self.comp_roll = 0.0
            self.comp_pitch = 0.0
            self.comp_yaw = 0.0
            self.prev_time = None
            
        elif self.filter_method == "adaptive_alpha":
            # Adaptive Alpha Filter - Changes strength based on movement
            self.base_alpha_accel = 0.7   # Higher base alpha for responsiveness
            self.base_alpha_gyro = 0.8
            self.min_alpha = 0.3
            self.max_alpha = 0.9
            self.change_threshold = 2.0   # Threshold for detecting significant changes
            self.filtered_ax = None
            self.filtered_ay = None
            self.filtered_az = None
            self.filtered_gx = None
            self.filtered_gy = None
            self.filtered_gz = None
            self.first_reading = True
            
        elif self.filter_method == "median_ma":
            # Median + Moving Average - Simple but effective
            self.window_size = 5
            self.median_window = 3
            self.accel_buffer_x = deque(maxlen=self.window_size)
            self.accel_buffer_y = deque(maxlen=self.window_size)
            self.accel_buffer_z = deque(maxlen=self.window_size)
            self.gyro_buffer_x = deque(maxlen=self.window_size)
            self.gyro_buffer_y = deque(maxlen=self.window_size)
            self.gyro_buffer_z = deque(maxlen=self.window_size)
            self.median_buffer_ax = deque(maxlen=self.median_window)
            self.median_buffer_ay = deque(maxlen=self.median_window)
            self.median_buffer_az = deque(maxlen=self.median_window)
            
        elif self.filter_method == "kalman":
            # Simple Kalman Filter for each axis
            self.setup_kalman_filters()

    def setup_kalman_filters(self):
        """Setup simple Kalman filters for accelerometer data"""
        # Kalman filter parameters
        self.Q = 0.01  # Process noise
        self.R = 0.1   # Measurement noise
        
        # Initialize Kalman states for accelerometer
        self.kalman_ax = {"x": 0, "P": 1, "K": 0}
        self.kalman_ay = {"x": 0, "P": 1, "K": 0}
        self.kalman_az = {"x": 0, "P": 1, "K": 0}
        
        # Gyro gets light filtering only
        self.gyro_alpha = 0.8

    def kalman_update(self, kalman_state, measurement):
        """Update Kalman filter state"""
        # Prediction
        kalman_state["P"] += self.Q
        
        # Update
        kalman_state["K"] = kalman_state["P"] / (kalman_state["P"] + self.R)
        kalman_state["x"] += kalman_state["K"] * (measurement - kalman_state["x"])
        kalman_state["P"] *= (1 - kalman_state["K"])
        
        return kalman_state["x"]

    def complementary_filter(self, ax, ay, az, gx, gy, gz):
        """
        Complementary filter - Combines gyro (short-term) and accel (long-term)
        Excellent for preserving terrain changes while filtering noise
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.prev_time is None:
            # Initialize with accelerometer
            self.comp_roll = math.atan2(ay, math.sqrt(ax*ax + az*az))
            self.comp_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            self.comp_yaw = 0.0
            self.prev_time = current_time
            return ax, ay, az, gx, gy, gz
        
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        # Integrate gyroscope for short-term changes
        gyro_roll = self.comp_roll + gx * dt
        gyro_pitch = self.comp_pitch + gy * dt
        gyro_yaw = self.comp_yaw + gz * dt
        
        # Calculate accelerometer angles for long-term reference
        accel_roll = math.atan2(ay, math.sqrt(ax*ax + az*az))
        accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        
        # Combine using complementary filter
        self.comp_roll = self.comp_alpha * gyro_roll + (1 - self.comp_alpha) * accel_roll
        self.comp_pitch = self.comp_alpha * gyro_pitch + (1 - self.comp_alpha) * accel_pitch
        self.comp_yaw = gyro_yaw  # Yaw from gyro only
        
        return ax, ay, az, gx, gy, gz

    def adaptive_alpha_filter(self, new_ax, new_ay, new_az, new_gx, new_gy, new_gz):
        """
        Adaptive alpha filter - Reduces filtering when significant changes detected
        """
        if self.first_reading:
            self.filtered_ax = new_ax
            self.filtered_ay = new_ay
            self.filtered_az = new_az
            self.filtered_gx = new_gx
            self.filtered_gy = new_gy
            self.filtered_gz = new_gz
            self.first_reading = False
            return new_ax, new_ay, new_az, new_gx, new_gy, new_gz
        
        # Calculate change magnitude
        accel_change = abs(new_ax - self.filtered_ax) + abs(new_ay - self.filtered_ay) + abs(new_az - self.filtered_az)
        gyro_change = abs(new_gx - self.filtered_gx) + abs(new_gy - self.filtered_gy) + abs(new_gz - self.filtered_gz)
        
        # Adapt alpha based on change magnitude
        if accel_change > self.change_threshold:
            alpha_accel = self.max_alpha  # Less filtering during changes
        else:
            alpha_accel = self.base_alpha_accel
            
        if gyro_change > self.change_threshold * 10:  # Gyro in deg/s
            alpha_gyro = self.max_alpha
        else:
            alpha_gyro = self.base_alpha_gyro
        
        # Apply adaptive filtering
        self.filtered_ax = alpha_accel * new_ax + (1 - alpha_accel) * self.filtered_ax
        self.filtered_ay = alpha_accel * new_ay + (1 - alpha_accel) * self.filtered_ay
        self.filtered_az = alpha_accel * new_az + (1 - alpha_accel) * self.filtered_az
        
        self.filtered_gx = alpha_gyro * new_gx + (1 - alpha_gyro) * self.filtered_gx
        self.filtered_gy = alpha_gyro * new_gy + (1 - alpha_gyro) * self.filtered_gy
        self.filtered_gz = alpha_gyro * new_gz + (1 - alpha_gyro) * self.filtered_gz
        
        return (self.filtered_ax, self.filtered_ay, self.filtered_az,
                self.filtered_gx, self.filtered_gy, self.filtered_gz)

    def median_moving_average_filter(self, new_ax, new_ay, new_az, new_gx, new_gy, new_gz):
        """
        Median filter + Moving average - Removes spikes, preserves trends
        """
        # Add to median filter first (removes spikes)
        self.median_buffer_ax.append(new_ax)
        self.median_buffer_ay.append(new_ay)
        self.median_buffer_az.append(new_az)
        
        if len(self.median_buffer_ax) >= self.median_window:
            med_ax = np.median(list(self.median_buffer_ax))
            med_ay = np.median(list(self.median_buffer_ay))
            med_az = np.median(list(self.median_buffer_az))
        else:
            med_ax, med_ay, med_az = new_ax, new_ay, new_az
        
        # Add to moving average buffers
        self.accel_buffer_x.append(med_ax)
        self.accel_buffer_y.append(med_ay)
        self.accel_buffer_z.append(med_az)
        self.gyro_buffer_x.append(new_gx)
        self.gyro_buffer_y.append(new_gy)
        self.gyro_buffer_z.append(new_gz)
        
        # Calculate moving averages
        if len(self.accel_buffer_x) >= self.window_size:
            filtered_ax = np.mean(list(self.accel_buffer_x))
            filtered_ay = np.mean(list(self.accel_buffer_y))
            filtered_az = np.mean(list(self.accel_buffer_z))
            filtered_gx = np.mean(list(self.gyro_buffer_x))
            filtered_gy = np.mean(list(self.gyro_buffer_y))
            filtered_gz = np.mean(list(self.gyro_buffer_z))
        else:
            filtered_ax, filtered_ay, filtered_az = med_ax, med_ay, med_az
            filtered_gx, filtered_gy, filtered_gz = new_gx, new_gy, new_gz
        
        return filtered_ax, filtered_ay, filtered_az, filtered_gx, filtered_gy, filtered_gz

    def kalman_filter(self, new_ax, new_ay, new_az, new_gx, new_gy, new_gz):
        """
        Kalman filter for accelerometer, light filter for gyro
        """
        # Apply Kalman filtering to accelerometer
        filtered_ax = self.kalman_update(self.kalman_ax, new_ax)
        filtered_ay = self.kalman_update(self.kalman_ay, new_ay)
        filtered_az = self.kalman_update(self.kalman_az, new_az)
        
        # Light filtering for gyroscope
        if not hasattr(self, 'prev_gx'):
            self.prev_gx = new_gx
            self.prev_gy = new_gy
            self.prev_gz = new_gz
        
        filtered_gx = self.gyro_alpha * new_gx + (1 - self.gyro_alpha) * self.prev_gx
        filtered_gy = self.gyro_alpha * new_gy + (1 - self.gyro_alpha) * self.prev_gy
        filtered_gz = self.gyro_alpha * new_gz + (1 - self.gyro_alpha) * self.prev_gz
        
        self.prev_gx = filtered_gx
        self.prev_gy = filtered_gy
        self.prev_gz = filtered_gz
        
        return filtered_ax, filtered_ay, filtered_az, filtered_gx, filtered_gy, filtered_gz

    def apply_filter(self, new_ax, new_ay, new_az, new_gx, new_gy, new_gz):
        """Apply the selected filtering method"""
        
        if self.filter_method == "complementary":
            return self.complementary_filter(new_ax, new_ay, new_az, new_gx, new_gy, new_gz)
        elif self.filter_method == "adaptive_alpha":
            return self.adaptive_alpha_filter(new_ax, new_ay, new_az, new_gx, new_gy, new_gz)
        elif self.filter_method == "median_ma":
            return self.median_moving_average_filter(new_ax, new_ay, new_az, new_gx, new_gy, new_gz)
        elif self.filter_method == "kalman":
            return self.kalman_filter(new_ax, new_ay, new_az, new_gx, new_gy, new_gz)
        else:
            return new_ax, new_ay, new_az, new_gx, new_gy, new_gz

    def estimate_orientation_from_accel(self, ax, ay, az):
        """Estimate roll and pitch from accelerometer data"""
        if self.filter_method == "complementary":
            # Use complementary filter results
            return self.comp_roll, self.comp_pitch, self.comp_yaw
        else:
            # Use accelerometer-based estimation
            roll = math.atan2(ay, math.sqrt(ax*ax + az*az))
            pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            yaw = 0.0
            return roll, pitch, yaw
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        
        return x, y, z, w

    def read_serial_data(self):
        try:
            if not hasattr(self, 'serial_port') or not self.serial_port.is_open:
                return
                
            line = self.serial_port.readline().decode('utf-8').strip()
            
            if not line:
                return
            
            if not line.startswith("imu:"):
                if self.debug_filtering:
                    self.get_logger().debug(f'Filtered out: {line[:50]}...')
                return

            parts = line.replace("imu: ", "").split(',')
            if len(parts) != 7:
                self.get_logger().warn(f'Invalid IMU data format: expected 7 parts, got {len(parts)}')
                return

            timestamp_ms = int(parts[0])
            raw_ax, raw_ay, raw_az = float(parts[1]), float(parts[2]), float(parts[3])
            raw_gx, raw_gy, raw_gz = float(parts[4]), float(parts[5]), float(parts[6])

            # Apply selected filtering method
            ax, ay, az, gx, gy, gz = self.apply_filter(
                raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz
            )

            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Linear acceleration in m/s^2
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            
            # Set covariance for linear acceleration
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01

            # Angular velocity in rad/s (convert from deg/s)
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = math.radians(gz)
            
            # Set covariance for angular velocity
            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.01

            # Estimate orientation
            roll, pitch, yaw = self.estimate_orientation_from_accel(ax, ay, az)
            
            # Convert to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
            
            # Set orientation quaternion
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            
            # Set orientation covariance
            if self.filter_method == "complementary":
                # Better orientation accuracy with complementary filter
                imu_msg.orientation_covariance[0] = 0.05  # roll variance
                imu_msg.orientation_covariance[4] = 0.05  # pitch variance
                imu_msg.orientation_covariance[8] = 0.5   # yaw variance
            else:
                imu_msg.orientation_covariance[0] = 0.1
                imu_msg.orientation_covariance[4] = 0.1
                imu_msg.orientation_covariance[8] = 1.0

            # Save data to CSV
            self.save_to_csv(
                timestamp_ms,
                raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz,
                ax, ay, az, gx, gy, gz,
                roll, pitch, yaw,
                qx, qy, qz, qw
            )

            # Publish the message
            self.publisher_.publish(imu_msg)
            
            # Debug log
            if timestamp_ms % 1000 < 100:
                self.get_logger().info(
                    f'[{self.filter_method}] Filtered IMU - '
                    f'Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}), '
                    f'Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f}), '
                    f'RPY: ({math.degrees(roll):.1f}°, {math.degrees(pitch):.1f}°, {math.degrees(yaw):.1f}°)'
                )

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
        except ValueError as e:
            self.get_logger().warn(f"Failed to parse IMU serial data: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        # Close CSV file
        if self.enable_csv_logging and self.csv_file:
            try:
                self.csv_file.close()
                self.get_logger().info(f'CSV file closed: {self.csv_filename}')
            except Exception as e:
                self.get_logger().error(f'Error closing CSV file: {e}')
        
        # Close serial port
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AdvancedSerialImuPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()