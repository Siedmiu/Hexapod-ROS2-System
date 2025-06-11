#!/usr/bin/env python3
 
"""
Description:
    This ROS 2 node subscribes to IMU sensor data and plots it in real-time.
    It demonstrates ROS concepts such as node creation, subscribing to sensor_msgs/Imu
    and real-time data visualization using matplotlib.
-------
Subscription Topics:
    /imu/data - sensor_msgs/Imu (Raw IMU data from sensor)
"""

import rclpy 
from rclpy.node import Node  

from sensor_msgs.msg import Imu  
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import time

class ImuPlotter(Node):
    """Create an IMU data plotter node."""
 
    def __init__(self):
        """Initialize the node."""
 
        # Initialize the node with a name
        super().__init__('imu_plotter')
 
        # Create a subscriber for IMU data
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)  # QoS profile with queue size 10
            
        # Data storage for plotting (using deque for efficient append/pop operations)
        self.max_points = 100  # Maximum number of points to display
        
        # Linear acceleration data
        self.accel_x = deque(maxlen=self.max_points)
        self.accel_y = deque(maxlen=self.max_points)
        self.accel_z = deque(maxlen=self.max_points)
        
        # Angular velocity data
        self.gyro_x = deque(maxlen=self.max_points)
        self.gyro_y = deque(maxlen=self.max_points)
        self.gyro_z = deque(maxlen=self.max_points)
        
        # Orientation data (quaternion converted to Euler angles)
        self.roll = deque(maxlen=self.max_points)
        self.pitch = deque(maxlen=self.max_points)
        self.yaw = deque(maxlen=self.max_points)
        
        # Timestamps
        self.timestamps = deque(maxlen=self.max_points)
        
        # Thread lock for data access
        self.data_lock = threading.Lock()
        
        # Initialize plotting
        self.setup_plots()
        
        # Log node startup
        self.get_logger().info('IMU Plotter node has started')
 
    def setup_plots(self):
        """Set up the matplotlib plots."""
        # Create figure with subplots
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('Real-time IMU Data', fontsize=16)
        
        # Configure acceleration plot
        self.axes[0].set_title('Linear Acceleration')
        self.axes[0].set_ylabel('Acceleration (m/s²)')
        self.axes[0].grid(True)
        self.accel_lines = {
            'x': self.axes[0].plot([], [], 'r-', label='X', linewidth=2)[0],
            'y': self.axes[0].plot([], [], 'g-', label='Y', linewidth=2)[0],
            'z': self.axes[0].plot([], [], 'b-', label='Z', linewidth=2)[0]
        }
        self.axes[0].legend()
        
        # Configure angular velocity plot
        self.axes[1].set_title('Angular Velocity')
        self.axes[1].set_ylabel('Angular Velocity (rad/s)')
        self.axes[1].grid(True)
        self.gyro_lines = {
            'x': self.axes[1].plot([], [], 'r-', label='X', linewidth=2)[0],
            'y': self.axes[1].plot([], [], 'g-', label='Y', linewidth=2)[0],
            'z': self.axes[1].plot([], [], 'b-', label='Z', linewidth=2)[0]
        }
        self.axes[1].legend()
        
        # Configure orientation plot
        self.axes[2].set_title('Orientation (Euler Angles)')
        self.axes[2].set_ylabel('Angle (radians)')
        self.axes[2].set_xlabel('Time (relative)')
        self.axes[2].grid(True)
        self.orientation_lines = {
            'roll': self.axes[2].plot([], [], 'r-', label='Roll', linewidth=2)[0],
            'pitch': self.axes[2].plot([], [], 'g-', label='Pitch', linewidth=2)[0],
            'yaw': self.axes[2].plot([], [], 'b-', label='Yaw', linewidth=2)[0]
        }
        self.axes[2].legend()
        
        # Adjust layout
        plt.tight_layout()
        
        # Start animation
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plots, interval=50, blit=False)
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
 
    def imu_callback(self, msg):
        """Process incoming IMU data.
        
        Args:
            msg (sensor_msgs/Imu): The IMU message received from the sensor
        """
        # Debug: Print that we received a message
        self.get_logger().info('Received IMU message!')
        
        with self.data_lock:
            # Get current time (relative to start)
            current_time = time.time()
            if not self.timestamps:
                self.start_time = current_time
            relative_time = current_time - getattr(self, 'start_time', current_time)
            
            # Store linear acceleration data
            self.accel_x.append(msg.linear_acceleration.x)
            self.accel_y.append(msg.linear_acceleration.y)
            self.accel_z.append(msg.linear_acceleration.z)
            
            # Store angular velocity data
            self.gyro_x.append(msg.angular_velocity.x)
            self.gyro_y.append(msg.angular_velocity.y)
            self.gyro_z.append(msg.angular_velocity.z)
            
            # Convert quaternion to Euler angles and store
            roll, pitch, yaw = self.quaternion_to_euler(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )
            self.roll.append(roll)
            self.pitch.append(pitch)
            self.yaw.append(yaw)
            
            # Store timestamp
            self.timestamps.append(relative_time)
            
            # Debug: Print data info
            self.get_logger().info(
                f'Data points stored: {len(self.timestamps)} | '
                f'Latest accel: ({msg.linear_acceleration.x:.2f}, '
                f'{msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f})'
            )
    
    def update_plots(self, frame):
        """Update the plots with new data."""
        with self.data_lock:
            if len(self.timestamps) < 2:
                return
            
            # Debug: Print update info
            if frame % 20 == 0:  # Print every 20 frames to avoid spam
                print(f"Updating plots - Data points: {len(self.timestamps)}")
            
            # Convert deques to lists for plotting
            times = list(self.timestamps)
            
            # Update acceleration plot
            self.accel_lines['x'].set_data(times, list(self.accel_x))
            self.accel_lines['y'].set_data(times, list(self.accel_y))
            self.accel_lines['z'].set_data(times, list(self.accel_z))
            
            # Update angular velocity plot
            self.gyro_lines['x'].set_data(times, list(self.gyro_x))
            self.gyro_lines['y'].set_data(times, list(self.gyro_y))
            self.gyro_lines['z'].set_data(times, list(self.gyro_z))
            
            # Update orientation plot
            self.orientation_lines['roll'].set_data(times, list(self.roll))
            self.orientation_lines['pitch'].set_data(times, list(self.pitch))
            self.orientation_lines['yaw'].set_data(times, list(self.yaw))
            
            # Auto-scale the plots
            for ax in self.axes:
                ax.relim()
                ax.autoscale_view()
        
        return (list(self.accel_lines.values()) + 
                list(self.gyro_lines.values()) + 
                list(self.orientation_lines.values()))
    
    def start_plotting(self):
        """Start the plotting - this should run in the main thread."""
        plt.show()

def main(args=None):
    """Main function to start the ROS 2 node.
 
    Args:
        args (List, optional): Command-line arguments. Defaults to None.
    """

    # Initialize ROS 2 communication
    rclpy.init(args=args)
 
    # Create an instance of the ImuPlotter node
    imu_plotter = ImuPlotter()
    
    # Start ROS spinning in a separate thread
    def spin_ros():
        try:
            rclpy.spin(imu_plotter)
        except Exception as e:
            print(f"ROS spinning error: {e}")
    
    ros_thread = threading.Thread(target=spin_ros)
    ros_thread.daemon = True
    ros_thread.start()
 
    try:
        # Start plotting in the main thread (this blocks until window is closed)
        imu_plotter.start_plotting()
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        imu_plotter.destroy_node()
        
        # Shutdown ROS 2 communication
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    # Execute the main function if the script is run directly
    main()