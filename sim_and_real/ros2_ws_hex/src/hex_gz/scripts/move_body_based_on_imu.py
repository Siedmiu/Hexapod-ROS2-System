#!/usr/bin/env python3
"""
IMU-Based Platform Leveling System for Hexapod Robot

This ROS 2 node reads IMU data and automatically adjusts the robot platform
to achieve 0,0,0 RPY orientation for stable walking on slopes.

Subscription Topics:
    /imu - sensor_msgs/Imu (IMU sensor data)
    /clock - rosgraph_msgs/Clock (Simulation time)

Publication Topics:
    /leg*_controller/joint_trajectory - trajectory_msgs/JointTrajectory
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rosgraph_msgs.msg import Clock
import time
import numpy as np
import threading
from collections import deque

# Joint limits for servo motors
JOINT_LIMITS = {
    'alfa_1': (-np.pi/4, np.pi/4),       # coxa joint: -45° to 45°
    'alfa_2': (-np.pi/3, np.pi/3),       # femur joint: -60° to 60°  
    'alfa_3': (-np.pi/2, np.pi/2)        # tibia joint: -90° to 90°
}

def clamp_angle(angle, min_angle, max_angle):
    """Clamp angle to specified limits"""
    return np.clip(angle, min_angle, max_angle)

def apply_joint_limits(alfa_1, alfa_2, alfa_3):
    """Apply joint limits to all servo angles"""
    alfa_1_limited = clamp_angle(alfa_1, JOINT_LIMITS['alfa_1'][0], JOINT_LIMITS['alfa_1'][1])
    alfa_2_limited = clamp_angle(alfa_2, JOINT_LIMITS['alfa_2'][0], JOINT_LIMITS['alfa_2'][1])
    alfa_3_limited = clamp_angle(alfa_3, JOINT_LIMITS['alfa_3'][0], JOINT_LIMITS['alfa_3'][1])
    
    return alfa_1_limited, alfa_2_limited, alfa_3_limited

def katy_serw(P3, l1, l2, l3):
    """Calculate servo angles with joint limits applied"""
    alfa_1 = np.arctan2(P3[1], P3[0])
    P1 = np.array([l1 * np.cos(alfa_1), l1 * np.sin(alfa_1), 0])
    d = np.sqrt((P3[0] - P1[0]) ** 2 + (P3[1] - P1[1]) ** 2 + (P3[2] - P1[2]) ** 2)

    cos_fi = (l2 ** 2 + l3 ** 2 - d ** 2) / (2 * l2 * l3)
    
    # Check if target is reachable
    if abs(cos_fi) > 1:
        print(f"Warning: Target unreachable, cos_fi = {cos_fi}")
        cos_fi = np.clip(cos_fi, -1, 1)
    
    fi = np.arccos(cos_fi)
    alfa_3 = np.deg2rad(180) - fi
    epsilon = np.arcsin(np.sin(fi) * l3 / d)
    tau = np.arctan2(P3[2] - P1[2], np.sqrt((P3[0] - P1[0]) ** 2 + (P3[1] - P1[1]) ** 2))
    alfa_2 = -(epsilon + tau)
    
    # Apply joint limits
    alfa_1, alfa_2, alfa_3 = apply_joint_limits(alfa_1, alfa_2, alfa_3)
    
    return [alfa_1, alfa_2, alfa_3]

def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    """Create 4x4 transformation matrix from body pose"""
    # Create individual rotation matrices
    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    R_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix
    R = R_yaw @ R_pitch @ R_roll
    
    # Create 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T

def body_ik_calculate_angles(fixed_foot_positions, body_pose, leg_attachments, leg_orientations, 
                           l1, l2, l3):
    """Calculate joint angles for all legs given fixed foot positions and body pose"""
    x_b, y_b, z_b, roll, pitch, yaw = body_pose
    
    # Create transformation matrix from world to body coordinates
    T_world_to_body = np.linalg.inv(create_transformation_matrix(x_b, y_b, z_b, roll, pitch, yaw))
    
    joint_angles = []
    
    for i in range(6):
        # Transform fixed foot position to body coordinates
        foot_world_homogeneous = np.append(fixed_foot_positions[i], 1)
        foot_body_homogeneous = T_world_to_body @ foot_world_homogeneous
        foot_body = foot_body_homogeneous[:3]
        
        # Calculate leg attachment position in current body frame
        leg_attach_body = leg_attachments[i]
        
        # Calculate leg vector from attachment to foot in body coordinates
        leg_vector_body = foot_body - leg_attach_body
        
        # Rotate leg vector to leg coordinate system
        leg_orientation = leg_orientations[i]
        rotation_to_leg = np.array([
            [np.cos(-leg_orientation), -np.sin(-leg_orientation), 0],
            [np.sin(-leg_orientation), np.cos(-leg_orientation), 0],
            [0, 0, 1]
        ])
        
        leg_vector_leg_coords = rotation_to_leg @ leg_vector_body
        
        # Calculate target position for leg IK
        target_position = leg_vector_leg_coords
        
        # Calculate joint angles using existing IK function
        try:
            angles = katy_serw(target_position, l1, l2, l3)
            joint_angles.append(angles)
        except:
            # If IK fails, use safe default angles
            default_angles = [0, np.radians(10), np.radians(80)]  # 0°, 10°, 80°
            safe_angles = apply_joint_limits(*default_angles)
            joint_angles.append(list(safe_angles))
            print(f"⚠️ IK failed for leg {i+1}, using default rest position")
    
    return np.array(joint_angles)

def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
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

class IMUPlatformLeveler(Node):
    """ROS 2 node for IMU-based platform leveling
    
    Reads IMU data and automatically moves the robot platform to achieve
    0,0,0 roll-pitch-yaw orientation by adjusting leg positions.
    """
    
    def __init__(self):
        super().__init__('imu_platform_leveler')
        self.get_logger().info('Inicjalizacja węzła do poziomowania platformy na podstawie IMU')
        
        # Leg kinematics parameters (from bi_gate)
        self.l1 = 0.17995 - 0.12184
        self.l2 = 0.30075 - 0.17995
        self.l3 = 0.50975 - 0.30075
        
        # Leg orientations (from bi_gate)
        self.leg_orientations = np.array([
            np.deg2rad(45), 0, np.deg2rad(-45), 
            np.deg2rad(180 + 45), np.deg2rad(180), np.deg2rad(180 - 45)
        ])
        
        # Leg attachments (simplified)
        self.leg_attachments = np.array([
            [ 0.073922, 0.055095 , 0.003148],
            [ 0.0978,  -0.00545,   0.003148],
            [ 0.067301, -0.063754, 0.003148],
            [-0.067301, -0.063754, 0.003148],
            [-0.0978,   -0.00545,  0.003148],
            [-0.073922,  0.055095, 0.003148],
        ])
        
        # Calculate rest position and fixed foot positions
        self.setup_rest_position()
        
        # IMU data storage
        self.max_imu_samples = 10  # Number of samples for smoothing
        self.imu_roll = deque(maxlen=self.max_imu_samples)
        self.imu_pitch = deque(maxlen=self.max_imu_samples)
        self.imu_yaw = deque(maxlen=self.max_imu_samples)
        self.imu_data_lock = threading.Lock()
        
        # Control parameters
        self.control_enabled = False
        self.target_roll = 0.0      # Desired roll angle (radians)
        self.target_pitch = 0.0     # Desired pitch angle (radians)
        self.target_yaw = 0.0       # Desired yaw angle (radians)
        
        # Control gains (proportional control only)
        self.gain_roll = 0.8        # Gain for roll correction
        self.gain_pitch = 0.8       # Gain for pitch correction
        self.gain_yaw = 0.3         # Gain for yaw correction (smaller for stability)
        
        # Maximum correction angles (safety limits)
        self.max_correction_roll = np.radians(15)   # ±15 degrees
        self.max_correction_pitch = np.radians(15)  # ±15 degrees
        self.max_correction_yaw = np.radians(10)    # ±10 degrees
        
        # Simulation time tracking
        self.sim_time = None
        
        # Create subscribers
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        self.clock_subscriber = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        
        # Create publishers for all leg controllers
        self.trajectory_publishers = {
            1: self.create_publisher(JointTrajectory, '/leg1_controller/joint_trajectory', 10),
            2: self.create_publisher(JointTrajectory, '/leg2_controller/joint_trajectory', 10),
            3: self.create_publisher(JointTrajectory, '/leg3_controller/joint_trajectory', 10),
            4: self.create_publisher(JointTrajectory, '/leg4_controller/joint_trajectory', 10),
            5: self.create_publisher(JointTrajectory, '/leg5_controller/joint_trajectory', 10),
            6: self.create_publisher(JointTrajectory, '/leg6_controller/joint_trajectory', 10)
        }
        
        # Joint names for each leg
        self.joint_names = {
            1: ['joint1_1', 'joint2_1', 'joint3_1'],
            2: ['joint1_2', 'joint2_2', 'joint3_2'],
            3: ['joint1_3', 'joint2_3', 'joint3_3'],
            4: ['joint1_4', 'joint2_4', 'joint3_4'],
            5: ['joint1_5', 'joint2_5', 'joint3_5'],
            6: ['joint1_6', 'joint2_6', 'joint3_6']
        }
        
        # Control timer (20 Hz control loop)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('IMU Platform Leveler node started')
    
    def setup_rest_position(self):
        """Setup rest position and calculate fixed foot positions"""
        # Rest angles (from bi_gate)
        alfa_1 = 0                    # 0°
        alfa_2 = np.radians(10)       # 10° 
        alfa_3 = np.radians(80)       # 80°
        
        # Calculate rest foot position
        P0 = np.array([0, 0, 0])
        P1 = P0 + np.array([self.l1 * np.cos(alfa_1), self.l1 * np.sin(alfa_1), 0])
        P2 = P1 + np.array([np.cos(alfa_1)*np.cos(alfa_2)*self.l2, 
                           np.sin(alfa_1)*np.cos(alfa_2)*self.l2, 
                           np.sin(alfa_2) * self.l2])
        P3 = P2 + np.array([np.cos(alfa_1)*np.cos(alfa_2 - alfa_3)*self.l3, 
                           np.sin(alfa_1)*np.cos(alfa_2 - alfa_3)*self.l3, 
                           np.sin(alfa_2 - alfa_3) * self.l3])
        
        x_start = self.l1 + self.l2 * np.cos(alfa_2) + self.l3 * np.sin(np.deg2rad(90) - alfa_2 - alfa_3)
        z_start = -(self.l2*np.sin(alfa_2) + self.l3 * np.cos(np.deg2rad(90) - alfa_2 - alfa_3))
        
        self.stopa_spoczynkowa = [x_start, 0, z_start]
        
        # Calculate fixed foot positions in world coordinates
        self.fixed_foot_positions_world = np.array([
            self.leg_attachments[i] + np.array([
                self.stopa_spoczynkowa[0] * np.cos(self.leg_orientations[i]) -
                self.stopa_spoczynkowa[1] * np.sin(self.leg_orientations[i]),
                self.stopa_spoczynkowa[0] * np.sin(self.leg_orientations[i]) +
                self.stopa_spoczynkowa[1] * np.cos(self.leg_orientations[i]),
                self.stopa_spoczynkowa[2]
            ]) for i in range(6)
        ])
        
        self.get_logger().info(f'Rest position calculated: foot at {self.stopa_spoczynkowa}')
    
    def clock_callback(self, msg):
        """Callback for simulation time"""
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
    
    def imu_callback(self, msg):
        """Process incoming IMU data"""
        # Convert quaternion to Euler angles
        roll, pitch, yaw = quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # Store IMU data with thread safety
        with self.imu_data_lock:
            self.imu_roll.append(roll)
            self.imu_pitch.append(pitch)
            self.imu_yaw.append(yaw)
        
        # Log IMU data occasionally
        if len(self.imu_roll) % 20 == 0:  # Every 20 samples
            self.get_logger().debug(
                f'IMU: roll={np.rad2deg(roll):.1f}°, '
                f'pitch={np.rad2deg(pitch):.1f}°, '
                f'yaw={np.rad2deg(yaw):.1f}°'
            )
    
    def get_smoothed_imu_data(self):
        """Get smoothed IMU data (moving average)"""
        with self.imu_data_lock:
            if len(self.imu_roll) == 0:
                return 0.0, 0.0, 0.0
            
            avg_roll = np.mean(list(self.imu_roll))
            avg_pitch = np.mean(list(self.imu_pitch))
            avg_yaw = np.mean(list(self.imu_yaw))
            
            return avg_roll, avg_pitch, avg_yaw
    
    def send_platform_correction(self, body_pose):
        """Send platform correction to all leg controllers"""
        # Calculate joint angles for the corrected body pose
        joint_angles = body_ik_calculate_angles(
            self.fixed_foot_positions_world,
            body_pose,
            self.leg_attachments,
            self.leg_orientations,
            self.l1, self.l2, self.l3
        )
        
        # Create and send trajectories
        duration = Duration()
        duration.sec = 0
        duration.nanosec = int(0.05 * 1e9)  # 50ms duration
        
        for leg_num in range(1, 7):
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names[leg_num]
            
            point = JointTrajectoryPoint()
            point.positions = [
                float(joint_angles[leg_num-1][0]),  # joint1
                float(joint_angles[leg_num-1][1]),  # joint2
                float(joint_angles[leg_num-1][2])   # joint3
            ]
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            point.time_from_start = duration
            
            trajectory.points.append(point)
            self.trajectory_publishers[leg_num].publish(trajectory)
    
    def control_loop(self):
        """Main control loop for platform leveling"""
        if not self.control_enabled or self.sim_time is None:
            return
        
        # Get current IMU data
        current_roll, current_pitch, current_yaw = self.get_smoothed_imu_data()
        
        if len(self.imu_roll) < 3:  # Wait for enough IMU data
            return
        
        # To achieve 0,0,0 RPY: apply opposite of current IMU reading
        # If IMU reads +10° roll, robot body should tilt -10° to compensate
        body_correction_roll = -current_roll * self.gain_roll
        body_correction_pitch = -current_pitch * self.gain_pitch  
        body_correction_yaw = -current_yaw * self.gain_yaw
        
        # Apply safety limits
        body_correction_roll = np.clip(body_correction_roll, -self.max_correction_roll, self.max_correction_roll)
        body_correction_pitch = np.clip(body_correction_pitch, -self.max_correction_pitch, self.max_correction_pitch)
        body_correction_yaw = np.clip(body_correction_yaw, -self.max_correction_yaw, self.max_correction_yaw)
        
        # Apply corrections to body pose to achieve 0,0,0 RPY
        corrected_body_pose = [
            0.0,                        # x translation
            0.0,                        # y translation  
            0.02,                       # z translation (small lift)
            body_correction_roll,       # roll correction to achieve 0° roll
            body_correction_pitch,      # pitch correction to achieve 0° pitch
            body_correction_yaw         # yaw correction to achieve 0° yaw
        ]
        
        # Send platform correction
        self.send_platform_correction(corrected_body_pose)
        
        # Log control status occasionally
        current_time = self.sim_time
        if int(current_time * 2) % 20 == 0:  # Every 10 seconds at 2Hz
            self.get_logger().info(
                f'IMU: roll={np.rad2deg(current_roll):.1f}°, pitch={np.rad2deg(current_pitch):.1f}°, yaw={np.rad2deg(current_yaw):.1f}° | '
                f'Body correction: roll={np.rad2deg(body_correction_roll):.1f}°, pitch={np.rad2deg(body_correction_pitch):.1f}°, yaw={np.rad2deg(body_correction_yaw):.1f}°'
            )
    
    def enable_control(self):
        """Enable platform leveling control"""
        self.control_enabled = True
        self.get_logger().info('Platform leveling control ENABLED')
    
    def disable_control(self):
        """Disable platform leveling control"""
        self.control_enabled = False
        self.get_logger().info('Platform leveling control DISABLED')
    
    def set_target_orientation(self, roll=0.0, pitch=0.0, yaw=0.0):
        """Set target platform orientation"""
        self.target_roll = roll
        self.target_pitch = pitch
        self.target_yaw = yaw
        self.get_logger().info(
            f'Target orientation set: roll={np.rad2deg(roll):.1f}°, '
            f'pitch={np.rad2deg(pitch):.1f}°, yaw={np.rad2deg(yaw):.1f}°'
        )
    
    def reset_to_rest_position(self):
        """Reset platform to rest position"""
        self.disable_control()
        
        # Send rest position
        rest_body_pose = [0.0, 0.0, 0.02, 0.0, 0.0, 0.0]
        self.send_platform_correction(rest_body_pose)
        
        self.get_logger().info('Platform reset to rest position')
    
    def run_leveling_demo(self):
        """Run a demonstration of platform leveling to achieve 0,0,0 RPY"""
        self.get_logger().info('Starting platform leveling demonstration...')
        
        # Wait for IMU data
        self.get_logger().info('Waiting for IMU data...')
        while len(self.imu_roll) < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Reset to rest position first
        self.reset_to_rest_position()
        time.sleep(2.0)
        
        # Set target level orientation (0,0,0 RPY)
        self.set_target_orientation(0.0, 0.0, 0.0)
        
        # Enable control
        self.enable_control()
        
        # Run for demonstration period
        demo_duration = 30.0  # 30 seconds
        start_time = time.time()
        
        self.get_logger().info(f'Running platform leveling for {demo_duration} seconds...')
        self.get_logger().info('Tilt the robot and watch it automatically move to achieve 0,0,0 RPY!')
        
        while time.time() - start_time < demo_duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Disable control and return to rest
        self.disable_control()
        self.reset_to_rest_position()
        
        self.get_logger().info('Platform leveling demonstration completed!')


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = IMUPlatformLeveler()
    
    try:
        # Wait for initialization
        print("Initializing IMU Platform Leveler...")
        time.sleep(2.0)
        
        # Run the leveling demonstration
        node.run_leveling_demo()
        
        # Keep node running for manual control if needed
        print("Demo finished. Node still running for manual control...")
        print("You can now manually tilt the robot to test the leveling system.")
        
        # Continue spinning
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()