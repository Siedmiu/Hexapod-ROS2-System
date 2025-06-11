#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math

class SerialImuPublisher(Node):
    def __init__(self):
        super().__init__('serial_imu_publisher')
        
        # Create publisher for IMU data - matches plotter subscription topic
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        
        # Initialize serial connection
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info('Serial connection established on /dev/ttyUSB0')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return
            
        # Timer to read serial data at 10 Hz
        self.timer = self.create_timer(0.1, self.read_serial_data)
        
        # Initialize variables for orientation estimation
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.get_logger().info('Serial IMU Publisher node started')

    def estimate_orientation_from_accel(self, ax, ay, az):
        """
        Estimate roll and pitch from accelerometer data.
        This provides basic orientation when no magnetometer/gyro integration is available.
        """
        # Calculate roll and pitch from accelerometer
        # Note: This assumes the accelerometer is measuring gravity when stationary
        roll = math.atan2(ay, math.sqrt(ax*ax + az*az))
        pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        
        # Yaw cannot be determined from accelerometer alone
        yaw = 0.0
        
        return roll, pitch, yaw
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        """
        # Calculate half angles
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        # Calculate quaternion components
        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        
        return x, y, z, w

    def read_serial_data(self):
        try:
            # Check if serial port is available
            if not hasattr(self, 'serial_port') or not self.serial_port.is_open:
                return
                
            line = self.serial_port.readline().decode('utf-8').strip()
            if not line.startswith("imu:"):
                return

            # Parse the serial data
            parts = line.replace("imu: ", "").split(',')
            if len(parts) != 7:
                self.get_logger().warn(f'Invalid data format: expected 7 parts, got {len(parts)}')
                return

            # Extract data
            timestamp_ms = int(parts[0])
            ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
            gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])

            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Linear acceleration in m/s^2
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            
            # Set covariance for linear acceleration
            imu_msg.linear_acceleration_covariance[0] = 0.01  # x variance
            imu_msg.linear_acceleration_covariance[4] = 0.01  # y variance
            imu_msg.linear_acceleration_covariance[8] = 0.01  # z variance

            # Angular velocity in rad/s (convert from deg/s)
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = math.radians(gz)
            
            # Set covariance for angular velocity
            imu_msg.angular_velocity_covariance[0] = 0.01  # x variance
            imu_msg.angular_velocity_covariance[4] = 0.01  # y variance
            imu_msg.angular_velocity_covariance[8] = 0.01  # z variance

            # Estimate orientation from accelerometer data
            # This provides basic roll/pitch estimation for the plotter
            roll, pitch, yaw = self.estimate_orientation_from_accel(ax, ay, az)
            
            # Convert to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
            
            # Set orientation quaternion
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            
            # Set orientation covariance (indicating limited accuracy since it's from accel only)
            imu_msg.orientation_covariance[0] = 0.1   # roll variance
            imu_msg.orientation_covariance[4] = 0.1   # pitch variance
            imu_msg.orientation_covariance[8] = 1.0   # yaw variance (high since not measurable from accel)

            # Publish the message
            self.publisher_.publish(imu_msg)
            
            # Debug log (reduce frequency to avoid spam)
            if timestamp_ms % 1000 < 100:  # Log roughly every second
                self.get_logger().info(
                    f'Published IMU data - Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}), '
                    f'Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f}), '
                    f'Orientation: R:{math.degrees(roll):.1f}° P:{math.degrees(pitch):.1f}°'
                )

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
        except ValueError as e:
            self.get_logger().warn(f"Failed to parse serial data: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SerialImuPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()