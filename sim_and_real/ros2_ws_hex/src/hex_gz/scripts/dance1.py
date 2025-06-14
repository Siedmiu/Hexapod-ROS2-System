#!/usr/bin/env python3
#code to move platform without moving foots - Enhanced Spider Movements

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import matplotlib.animation as animation

matplotlib.use('TkAgg')

# Joint limits for servo motors - updated to accommodate rest position
JOINT_LIMITS = {
    'alfa_1': (-np.pi/4, np.pi/4),       # coxa joint: -45° to 45°
    'alfa_2': (-np.pi/3, np.pi/3),       # femur joint: -60° to 60°  
    'alfa_3': (-np.pi/2, np.pi/2)        # tibia joint: -90° to 90° (expanded to include 80°)
}

def clamp_angle(angle, min_angle, max_angle):
    """Clamp angle to specified limits"""
    return np.clip(angle, min_angle, max_angle)

def apply_joint_limits(alfa_1, alfa_2, alfa_3):
    """Apply joint limits to all servo angles"""
    alfa_1_limited = clamp_angle(alfa_1, JOINT_LIMITS['alfa_1'][0], JOINT_LIMITS['alfa_1'][1])
    alfa_2_limited = clamp_angle(alfa_2, JOINT_LIMITS['alfa_2'][0], JOINT_LIMITS['alfa_2'][1])
    alfa_3_limited = clamp_angle(alfa_3, JOINT_LIMITS['alfa_3'][0], JOINT_LIMITS['alfa_3'][1])
    
    # Log warnings if angles were clamped
    if abs(alfa_1 - alfa_1_limited) > 1e-6:
        print(f"Warning: alfa_1 clamped from {np.rad2deg(alfa_1):.1f}° to {np.rad2deg(alfa_1_limited):.1f}°")
    if abs(alfa_2 - alfa_2_limited) > 1e-6:
        print(f"Warning: alfa_2 clamped from {np.rad2deg(alfa_2):.1f}° to {np.rad2deg(alfa_2_limited):.1f}°")
    if abs(alfa_3 - alfa_3_limited) > 1e-6:
        print(f"Warning: alfa_3 clamped from {np.rad2deg(alfa_3):.1f}° to {np.rad2deg(alfa_3_limited):.1f}°")
    
    return alfa_1_limited, alfa_2_limited, alfa_3_limited

def katy_serw(P3, l1, l2, l3):
    """Calculate servo angles with joint limits applied - updated from bi_gate"""
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

def polozenie_przegub_1(l1, alfa1, przyczep):
    return np.array([l1 * np.cos(alfa1) + przyczep[0], l1 * np.sin(alfa1) + przyczep[1], przyczep[2]])

def polozenie_przegub_2(l1, l2, alfa1, alfa2, przyczep):
    return polozenie_przegub_1(l1, alfa1, przyczep) + np.array(
        [l2 * np.cos(alfa1) * np.cos(alfa2), l2 * np.sin(alfa1) * np.cos(alfa2), l2 * np.sin(alfa2)])

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
    """
    Calculate joint angles for all legs given fixed foot positions and body pose
    Joint limits are automatically applied within the katy_serw function.
    """
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
        
        # Calculate target position for leg IK (relative to leg attachment point)
        target_position = leg_vector_leg_coords
        
        # Calculate joint angles using existing IK function (limits applied automatically)
        try:
            angles = katy_serw(target_position, l1, l2, l3)
            joint_angles.append(angles)
        except:
            # If IK fails, use safe default angles within limits (same as rest position)
            default_angles = [0, np.radians(10), np.radians(80)]  # 0°, 10°, 80°
            # Apply limits to default angles too
            safe_angles = apply_joint_limits(*default_angles)
            joint_angles.append(list(safe_angles))
            print(f"⚠️ IK failed for leg {i+1}, using default rest position: {np.rad2deg(safe_angles)}")
    
    return np.array(joint_angles)

def generate_spider_movement_sequence(movement_type='up_down', duration=3.0, amplitude=0.04):
    """
    Generate enhanced spider movement sequences with many different movement patterns
    """
    num_frames = int(duration * 20)  # 20 Hz
    t = np.linspace(0, duration, num_frames)
    
    body_poses = []
    
    if movement_type == 'up_down':
        # Simple up-down movement like a spider breathing
        for i in range(num_frames):
            x = 0
            y = 0
            z = amplitude * 0.8 * np.sin(2 * np.pi * t[i] * 0.7) + amplitude * 0.2
            roll = 0
            pitch = 0
            yaw = 0
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'circle_left':
        # Circular movement to the left
        radius = amplitude * 0.8
        for i in range(num_frames):
            angle = 2 * np.pi * t[i] / duration
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = amplitude * 0.3 * np.sin(2 * angle) + amplitude * 0.2
            roll = amplitude * 0.5 * np.sin(angle)
            pitch = amplitude * 0.5 * np.cos(angle)
            yaw = angle * 0.3
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'circle_right':
        # Circular movement to the right (reverse direction)
        radius = amplitude * 0.8
        for i in range(num_frames):
            angle = -2 * np.pi * t[i] / duration  # Negative for right direction
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = amplitude * 0.3 * np.sin(2 * angle) + amplitude * 0.2
            roll = amplitude * 0.5 * np.sin(angle)
            pitch = amplitude * 0.5 * np.cos(angle)
            yaw = angle * 0.3
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'zigzag':
        # Zigzag movement like a hunting spider
        for i in range(num_frames):
            x = amplitude * 0.7 * np.sin(4 * np.pi * t[i] / duration)
            y = amplitude * 0.9 * np.sin(2 * np.pi * t[i] / duration)
            z = amplitude * 0.4 * np.abs(np.sin(3 * np.pi * t[i] / duration)) + amplitude * 0.1
            roll = amplitude * 0.8 * np.sin(3 * np.pi * t[i] / duration)
            pitch = amplitude * 0.6 * np.sin(2.5 * np.pi * t[i] / duration)
            yaw = amplitude * 1.2 * np.sin(1.5 * np.pi * t[i] / duration)
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'spiral':
        # Spiral movement outward and inward
        for i in range(num_frames):
            spiral_t = t[i] / duration * 4 * np.pi  # 2 full spirals
            radius = amplitude * 0.6 * (1 + np.sin(2 * np.pi * t[i] / duration)) / 2
            x = radius * np.cos(spiral_t)
            y = radius * np.sin(spiral_t)
            z = amplitude * 0.5 * np.sin(6 * np.pi * t[i] / duration) + amplitude * 0.3
            roll = amplitude * 0.4 * np.sin(spiral_t)
            pitch = amplitude * 0.4 * np.cos(spiral_t)
            yaw = spiral_t * 0.2
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'tilt_forward_back':
        # Tilting forward and backward like looking around
        for i in range(num_frames):
            x = 0
            y = 0
            z = amplitude * 0.2 * np.sin(2 * np.pi * t[i] / duration) + amplitude * 0.15
            roll = 0
            pitch = amplitude * 2.0 * np.sin(np.pi * t[i] / duration)  # Bigger pitch movement
            yaw = 0
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'tilt_left_right':
        # Tilting left and right like a spider examining something
        for i in range(num_frames):
            x = 0
            y = 0
            z = amplitude * 0.2 * np.sin(3 * np.pi * t[i] / duration) + amplitude * 0.15
            roll = amplitude * 2.5 * np.sin(1.5 * np.pi * t[i] / duration)  # Bigger roll movement
            pitch = 0
            yaw = 0
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'figure8':
        # Figure 8 movement like a curious spider
        for i in range(num_frames):
            angle = 2 * np.pi * t[i] / duration
            x = amplitude * 0.8 * np.sin(angle)
            y = amplitude * 0.8 * np.sin(angle) * np.cos(angle)
            z = amplitude * 0.3 * np.sin(2 * angle) + amplitude * 0.2
            roll = amplitude * 0.6 * np.sin(2 * angle)
            pitch = amplitude * 0.4 * np.cos(angle)
            yaw = amplitude * 0.8 * np.sin(angle)
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'shake':
        # Quick shaking movement like an agitated spider
        for i in range(num_frames):
            x = amplitude * 0.3 * np.sin(12 * np.pi * t[i] / duration)
            y = amplitude * 0.3 * np.sin(15 * np.pi * t[i] / duration)
            z = amplitude * 0.2 * np.abs(np.sin(10 * np.pi * t[i] / duration)) + amplitude * 0.1
            roll = amplitude * 0.8 * np.sin(14 * np.pi * t[i] / duration)
            pitch = amplitude * 0.8 * np.sin(11 * np.pi * t[i] / duration)
            yaw = amplitude * 1.0 * np.sin(13 * np.pi * t[i] / duration)
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'crouch_rise':
        # Crouching down and rising up like a spider preparing to jump
        for i in range(num_frames):
            progress = t[i] / duration
            if progress < 0.3:  # Crouch down
                z_factor = 1 - 3 * progress
            elif progress < 0.7:  # Stay low
                z_factor = 0.1
            else:  # Rise up
                z_factor = 0.1 + 3 * (progress - 0.7)
            
            x = 0
            y = 0
            z = amplitude * 0.6 * z_factor
            roll = 0
            pitch = amplitude * 0.5 * (0.5 - z_factor)  # Pitch forward when low
            yaw = 0
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'look_around':
        # Looking around movement with head/body rotation
        for i in range(num_frames):
            progress = t[i] / duration
            if progress < 0.25:  # Look left
                yaw_target = amplitude * 2.0
            elif progress < 0.5:  # Look forward
                yaw_target = 0
            elif progress < 0.75:  # Look right
                yaw_target = -amplitude * 2.0
            else:  # Look forward again
                yaw_target = 0
            
            x = 0
            y = 0
            z = amplitude * 0.15 + amplitude * 0.1 * np.sin(8 * np.pi * t[i] / duration)
            roll = amplitude * 0.3 * np.sin(4 * np.pi * t[i] / duration)
            pitch = amplitude * 0.2 * np.sin(3 * np.pi * t[i] / duration)
            yaw = yaw_target * np.sin(np.pi * (progress % 0.25) / 0.25)
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'wave_motion':
        # Wave-like motion through the body
        for i in range(num_frames):
            wave_freq = 2.0
            x = amplitude * 0.4 * np.sin(2 * np.pi * wave_freq * t[i])
            y = amplitude * 0.3 * np.sin(2 * np.pi * wave_freq * t[i] + np.pi/4)
            z = amplitude * 0.5 * np.sin(2 * np.pi * wave_freq * t[i] + np.pi/2) + amplitude * 0.2
            roll = amplitude * 0.8 * np.sin(2 * np.pi * wave_freq * t[i] + np.pi/3)
            pitch = amplitude * 0.6 * np.sin(2 * np.pi * wave_freq * t[i] + np.pi/6)
            yaw = amplitude * 1.0 * np.sin(2 * np.pi * wave_freq * t[i] + np.pi/8)
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    else:  # Default: gentle oscillation
        for i in range(num_frames):
            x = amplitude * 0.3 * np.sin(2 * np.pi * t[i] * 0.5)
            y = amplitude * 0.2 * np.sin(2 * np.pi * t[i] * 0.3)
            z = amplitude * 0.4 * np.sin(2 * np.pi * t[i]) + amplitude * 0.2
            roll = amplitude * 0.5 * np.sin(2 * np.pi * t[i] * 0.7)
            pitch = amplitude * 0.6 * np.sin(2 * np.pi * t[i] * 0.5)
            yaw = amplitude * 0.8 * np.sin(2 * np.pi * t[i] * 0.3)
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    return np.array(body_poses)

def generate_body_movement_servo_angles(movement_type='up_down', duration=3.0, amplitude=0.04):
    """
    Generate body movement as servo angles array compatible with the existing ROS system
    Returns wychyly_serw_podczas_ruchu format for body movements
    """
    # Generate body poses
    body_poses = generate_spider_movement_sequence(movement_type, duration, amplitude)
    
    # Calculate servo angles for each body pose
    wychyly_serw_body_movement = []
    
    for body_pose in body_poses:
        # Calculate joint angles for current body pose
        joint_angles = body_ik_calculate_angles(
            fixed_foot_positions_world,
            body_pose,
            leg_attachments,
            leg_orientations,
            l1, l2, l3
        )
        wychyly_serw_body_movement.append(joint_angles)
    
    # Convert to same format as walking: [leg][step][joint]
    wychyly_serw_body_movement = np.array(wychyly_serw_body_movement)
    # Transpose to match format: [leg][step][joint] instead of [step][leg][joint]
    wychyly_serw_body_movement = np.transpose(wychyly_serw_body_movement, (1, 0, 2))
    
    return wychyly_serw_body_movement

l1 = 0.17995 - 0.12184
l2 = 0.30075 - 0.17995
l3 = 0.50975 - 0.30075

# Updated leg attachments and orientations to match bi_gate style
# Using the nachylenia_nog_do_bokow_platformy_pajaka from bi_gate
nachylenia_nog_do_bokow_platformy_pajaka = np.array([
    np.deg2rad(45), 0, np.deg2rad(-45), np.deg2rad(180 + 45), np.deg2rad(180), np.deg2rad(180 - 45)
])

# Simplified leg attachments - estimate based on bi_gate approach
leg_attachments = np.array([
    [ 0.073922, 0.055095 , 0.003148],  # Keep existing values but will use bi_gate approach
    [ 0.0978,  -0.00545,   0.003148],
    [ 0.067301, -0.063754, 0.003148],
    [-0.067301, -0.063754, 0.003148],
    [-0.0978,   -0.00545,  0.003148],
    [-0.073922,  0.055095, 0.003148],
])

# Use leg orientations from bi_gate
leg_orientations = nachylenia_nog_do_bokow_platformy_pajaka

# Calculate rest position as in bi_gate
alfa_1 = 0                    # 0°
alfa_2 = np.radians(10)       # 10° 
alfa_3 = np.radians(80)       # 80°

P0 = np.array([0, 0, 0])
P1 = P0 + np.array([l1 * np.cos(alfa_1), l1 *np.sin(alfa_1), 0])
P2 = P1 + np.array([np.cos(alfa_1)*np.cos(alfa_2)*l2,np.sin(alfa_1)*np.cos(alfa_2)*l2, np.sin(alfa_2) * l2])
P3 = P2 + np.array([np.cos(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_2 - alfa_3) * l3])

# Use the same foot rest position as bi_gate
x_start = l1 + l2 * np.cos(alfa_2) + l3 * np.sin(np.deg2rad(90) - alfa_2 - alfa_3)  # poczatkowe wychylenie nogi pajaka w osi x
z_start = -(l2*np.sin(alfa_2) + l3 * np.cos(np.deg2rad(90) - alfa_2 - alfa_3))  # poczatkowy z

stopa_spoczynkowa = [x_start, 0, z_start]
wysokosc_start = -stopa_spoczynkowa[2]

print(f"Kąty spoczynkowe: alfa_1={np.rad2deg(alfa_1):.1f}°, alfa_2={np.rad2deg(alfa_2):.1f}°, alfa_3={np.rad2deg(alfa_3):.1f}°")

# Calculate fixed foot positions in world coordinates for body movement - using bi_gate approach
fixed_foot_positions_world = np.array([
    leg_attachments[i] + np.array([
        stopa_spoczynkowa[0] * np.cos(leg_orientations[i]) -
        stopa_spoczynkowa[1] * np.sin(leg_orientations[i]),

        stopa_spoczynkowa[0] * np.sin(leg_orientations[i]) +
        stopa_spoczynkowa[1] * np.cos(leg_orientations[i]),

        stopa_spoczynkowa[2]
    ]) for i in range(6)
])

# Polozenie spoczynkowe stop
polozenie_spoczynkowe_stop = fixed_foot_positions_world

# Generate different spider movement sequences
print("Generating enhanced spider movement sequences...")

# Generate all the spider movements with varying parameters
print("🕷️ Generowanie sekwencji ruchów pajączka...")

wychyly_serw_up_down = generate_body_movement_servo_angles('up_down', duration=3.0, amplitude=0.04)
wychyly_serw_circle_left = generate_body_movement_servo_angles('circle_left', duration=4.0, amplitude=0.035)
wychyly_serw_circle_right = generate_body_movement_servo_angles('circle_right', duration=4.0, amplitude=0.035)
wychyly_serw_zigzag = generate_body_movement_servo_angles('zigzag', duration=3.5, amplitude=0.03)
wychyly_serw_spiral = generate_body_movement_servo_angles('spiral', duration=5.0, amplitude=0.025)
wychyly_serw_tilt_forward_back = generate_body_movement_servo_angles('tilt_forward_back', duration=3.0, amplitude=0.03)
wychyly_serw_tilt_left_right = generate_body_movement_servo_angles('tilt_left_right', duration=3.0, amplitude=0.03)
wychyly_serw_figure8 = generate_body_movement_servo_angles('figure8', duration=4.0, amplitude=0.03)
wychyly_serw_shake = generate_body_movement_servo_angles('shake', duration=2.0, amplitude=0.02)
wychyly_serw_crouch_rise = generate_body_movement_servo_angles('crouch_rise', duration=4.0, amplitude=0.045)
wychyly_serw_look_around = generate_body_movement_servo_angles('look_around', duration=4.0, amplitude=0.03)
wychyly_serw_wave_motion = generate_body_movement_servo_angles('wave_motion', duration=3.0, amplitude=0.03)

print("🕷️ Wszystkie sekwencje pajączka wygenerowane!")

# Add verification of rest angles
print(f"\n✅ WERYFIKACJA KĄTÓW SPOCZYNKOWYCH:")
print(f"alfa_1 = {alfa_1:.4f} rad = {np.rad2deg(alfa_1):6.1f}°")
print(f"alfa_2 = {alfa_2:.4f} rad = {np.rad2deg(alfa_2):6.1f}°") 
print(f"alfa_3 = {alfa_3:.4f} rad = {np.rad2deg(alfa_3):6.1f}°")

# Check if rest angles are within limits
rest_angles_ok = True
if not (JOINT_LIMITS['alfa_1'][0] <= alfa_1 <= JOINT_LIMITS['alfa_1'][1]):
    print(f"❌ PROBLEM: alfa_1 poza limitami!")
    rest_angles_ok = False
if not (JOINT_LIMITS['alfa_2'][0] <= alfa_2 <= JOINT_LIMITS['alfa_2'][1]):
    print(f"❌ PROBLEM: alfa_2 poza limitami!")
    rest_angles_ok = False
if not (JOINT_LIMITS['alfa_3'][0] <= alfa_3 <= JOINT_LIMITS['alfa_3'][1]):
    print(f"❌ PROBLEM: alfa_3 poza limitami!")
    rest_angles_ok = False

if rest_angles_ok:
    print("✅ Wszystkie kąty spoczynkowe w dozwolonych granicach")

class SpiderLegSequencePlayer(Node):
    def __init__(self, debug_joints=True):
        super().__init__('spider_leg_sequence_player')
        self.get_logger().info('🕷️ Inicjalizacja węzła pajączka do sekwencji ruchów platformy')
        
        # Debug flag for joint position printing
        self.debug_joints = debug_joints
        
        # Wydawcy dla kontrolerów wszystkich nóg
        self.trajectory_publishers = {
            1: self.create_publisher(JointTrajectory, '/leg1_controller/joint_trajectory', 10),
            2: self.create_publisher(JointTrajectory, '/leg2_controller/joint_trajectory', 10),
            3: self.create_publisher(JointTrajectory, '/leg3_controller/joint_trajectory', 10),
            4: self.create_publisher(JointTrajectory, '/leg4_controller/joint_trajectory', 10),
            5: self.create_publisher(JointTrajectory, '/leg5_controller/joint_trajectory', 10),
            6: self.create_publisher(JointTrajectory, '/leg6_controller/joint_trajectory', 10)
        }

        # Listy stawów dla nóg
        self.joint_names = {
            1: ['joint1_1', 'joint2_1', 'joint3_1'],
            2: ['joint1_2', 'joint2_2', 'joint3_2'],
            3: ['joint1_3', 'joint2_3', 'joint3_3'],
            4: ['joint1_4', 'joint2_4', 'joint3_4'],
            5: ['joint1_5', 'joint2_5', 'joint3_5'],
            6: ['joint1_6', 'joint2_6', 'joint3_6']
        }

    def print_joint_positions(self, wychyly_serw_array, step_index, sequence_name=""):
        """Print current joint positions for debugging"""
        if not self.debug_joints:
            return
            
        print(f"\n=== POZYCJE JOINTÓW - Krok {step_index} ({sequence_name}) ===")
        
        for leg_num in range(1, 7):
            joint_values = wychyly_serw_array[leg_num-1][step_index]
            
            print(f"Noga {leg_num}:")
            print(f"  Joint1 (alfa_1): {joint_values[0]:.4f} rad = {np.rad2deg(joint_values[0]):6.1f}°")
            print(f"  Joint2 (alfa_2): {joint_values[1]:.4f} rad = {np.rad2deg(joint_values[1]):6.1f}°")
            print(f"  Joint3 (alfa_3): {joint_values[2]:.4f} rad = {np.rad2deg(joint_values[2]):6.1f}°")
            
            # Check limits
            limits_ok = True
            if not (JOINT_LIMITS['alfa_1'][0] <= joint_values[0] <= JOINT_LIMITS['alfa_1'][1]):
                print(f"    ⚠️  Joint1 POZA GRANICAMI! Dozwolone: {np.rad2deg(JOINT_LIMITS['alfa_1'][0]):.1f}° do {np.rad2deg(JOINT_LIMITS['alfa_1'][1]):.1f}°")
                limits_ok = False
            if not (JOINT_LIMITS['alfa_2'][0] <= joint_values[1] <= JOINT_LIMITS['alfa_2'][1]):
                print(f"    ⚠️  Joint2 POZA GRANICAMI! Dozwolone: {np.rad2deg(JOINT_LIMITS['alfa_2'][0]):.1f}° do {np.rad2deg(JOINT_LIMITS['alfa_2'][1]):.1f}°")
                limits_ok = False
            if not (JOINT_LIMITS['alfa_3'][0] <= joint_values[2] <= JOINT_LIMITS['alfa_3'][1]):
                print(f"    ⚠️  Joint3 POZA GRANICAMI! Dozwolone: {np.rad2deg(JOINT_LIMITS['alfa_3'][0]):.1f}° do {np.rad2deg(JOINT_LIMITS['alfa_3'][1]):.1f}°")
                limits_ok = False
                
            if limits_ok:
                print(f"    ✅ Wszystkie jointy w granicach")
        
        print("=" * 50)

    def check_sequence_validity(self, wychyly_serw_array, sequence_name=""):
        """Check if entire sequence has valid joint positions"""
        print(f"\n🔍 SPRAWDZANIE SEKWENCJI: {sequence_name}")
        print(f"Liczba kroków: {len(wychyly_serw_array[0])}")
        
        problems_found = False
        for step in range(len(wychyly_serw_array[0])):
            step_problems = []
            for leg_num in range(6):
                joint_values = wychyly_serw_array[leg_num][step]
                
                if not (JOINT_LIMITS['alfa_1'][0] <= joint_values[0] <= JOINT_LIMITS['alfa_1'][1]):
                    step_problems.append(f"Noga{leg_num+1}-Joint1: {np.rad2deg(joint_values[0]):.1f}°")
                if not (JOINT_LIMITS['alfa_2'][0] <= joint_values[1] <= JOINT_LIMITS['alfa_2'][1]):
                    step_problems.append(f"Noga{leg_num+1}-Joint2: {np.rad2deg(joint_values[1]):.1f}°")
                if not (JOINT_LIMITS['alfa_3'][0] <= joint_values[2] <= JOINT_LIMITS['alfa_3'][1]):
                    step_problems.append(f"Noga{leg_num+1}-Joint3: {np.rad2deg(joint_values[2]):.1f}°")
            
            if step_problems:
                print(f"⚠️  Krok {step}: {', '.join(step_problems)}")
                problems_found = True
        
        if not problems_found:
            print(f"✅ Sekwencja {sequence_name} - wszystkie pozycje w granicach!")
        else:
            print(f"❌ Sekwencja {sequence_name} ma problemy z granicami jointów!")
        print("-" * 50)

    def send_trajectory_to_all_legs_at_step(self, wychyly_serw_array, step_index, duration_sec=0.1, sequence_name=""):
        """
        Wysyła trajektorię do kontrolerów wszystkich nóg jednocześnie
        używając wartości z tablicy wychyly_serw_array dla danego kroku
        """
        self.get_logger().info(f'🕷️ Wysyłam trajektorię dla kroku {step_index}')
        
        # Sprawdź, czy indeks jest prawidłowy
        if step_index >= len(wychyly_serw_array[0]):
            self.get_logger().error(f'Indeks kroku {step_index} jest poza zakresem!')
            return False
        
        # Print joint positions for debugging
        self.print_joint_positions(wychyly_serw_array, step_index, sequence_name)
        
        # Ustaw czas trwania ruchu
        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        # Dla każdej nogi przygotuj i wyślij trajektorię
        for leg_num in range(1, 7):
            # Utwórz wiadomość trajektorii dla nogi
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names[leg_num]
            
            # Utwórz punkt trajektorii
            point = JointTrajectoryPoint()
            
            # Pobierz wartości z tablicy (indeks nogi to leg_num-1)
            joint_values = wychyly_serw_array[leg_num-1][step_index]
            
            point.positions = [
                float(joint_values[0]),  # joint1
                float(joint_values[1]),  # joint2
                float(joint_values[2])   # joint3
            ]
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            
            # Ustaw czas trwania ruchu
            point.time_from_start = duration
            
            # Dodaj punkt do trajektorii
            trajectory.points.append(point)
            
            # Wyślij trajektorię
            self.trajectory_publishers[leg_num].publish(trajectory)
        
        self.get_logger().info('🕷️ Wysłano trajektorie dla wszystkich nóg')
        return True
    
    def execute_sequence(self, wychyly_serw_array, sequence_name="sequence", start_step=0, end_step=None, step_duration=0.05):
        """
        Wykonanie sekwencji ruchów dla wszystkich nóg równocześnie
        używając wartości z tablicy wychyly_serw_array
        """
        self.get_logger().info(f'🕷️ Rozpoczynam sekwencję: {sequence_name}')
        
        # Check sequence validity first
        self.check_sequence_validity(wychyly_serw_array, sequence_name)
        
        # Jeśli nie podano end_step, użyj całej tablicy
        if end_step is None:
            end_step = len(wychyly_serw_array[0])
        
        # Przejście do pozycji początkowej (pierwszy punkt w tablicy)
        self.send_trajectory_to_all_legs_at_step(wychyly_serw_array, start_step, duration_sec=0.2, sequence_name=sequence_name)
        self.get_logger().info('🕷️ Oczekiwanie na wykonanie początkowego ruchu...')
        time.sleep(0.2)
        
        # Wykonanie sekwencji ruchów
        for step in range(start_step + 1, end_step):
            # Print joint positions every step for first 5 steps, then every 10 steps
            if step <= 5 or step % 15 == 0:
                self.send_trajectory_to_all_legs_at_step(wychyly_serw_array, step, duration_sec=step_duration, sequence_name=sequence_name)
            else:
                # Silent mode for intermediate steps
                old_debug = self.debug_joints
                self.debug_joints = False
                self.send_trajectory_to_all_legs_at_step(wychyly_serw_array, step, duration_sec=step_duration, sequence_name=sequence_name)
                self.debug_joints = old_debug
                
            if step % 25 == 0:  # Log every 25 steps
                self.get_logger().info(f'🕷️ {sequence_name}: krok {step}/{end_step}, oczekiwanie {step_duration}s...')
            time.sleep(step_duration)
        
        self.get_logger().info(f'🕷️ Sekwencja {sequence_name} zakończona')

    def execute_complete_spider_dance_sequence(self):
        """Execute complete spider dance sequence with all enhanced movement types"""
        self.get_logger().info('🕷️ Rozpoczynam kompletną sekwencję tańca pajączka!')
        
        # Pre-check all sequences
        print("\n🔍 SPRAWDZANIE WSZYSTKICH SEKWENCJI PAJĄCZKA PRZED ROZPOCZĘCIEM:")
        self.check_sequence_validity(wychyly_serw_up_down, "🕷️ podnoszenie/opuszczanie")
        self.check_sequence_validity(wychyly_serw_circle_left, "🕷️ krążenie w lewo")
        self.check_sequence_validity(wychyly_serw_circle_right, "🕷️ krążenie w prawo")
        self.check_sequence_validity(wychyly_serw_zigzag, "🕷️ ruch zigzag")
        self.check_sequence_validity(wychyly_serw_spiral, "🕷️ ruch spiralny")
        self.check_sequence_validity(wychyly_serw_tilt_forward_back, "🕷️ pochylanie przód-tył")
        self.check_sequence_validity(wychyly_serw_tilt_left_right, "🕷️ pochylanie lewo-prawo")
        self.check_sequence_validity(wychyly_serw_figure8, "🕷️ ruch ósemkowy")
        self.check_sequence_validity(wychyly_serw_shake, "🕷️ potrząsanie")
        self.check_sequence_validity(wychyly_serw_crouch_rise, "🕷️ kucanie-wstawanie")
        self.check_sequence_validity(wychyly_serw_look_around, "🕷️ rozglądanie się")
        self.check_sequence_validity(wychyly_serw_wave_motion, "🕷️ ruch falowy")
        
        # Wait for initialization
        self.get_logger().info('🕷️ Czekam na inicjalizację pajączka...')
        time.sleep(2.0)
        
        # 1. Start with gentle up-down movement
        self.get_logger().info('🕷️ --- Faza 1: Podnoszenie i opuszczanie platformy ---')
        self.execute_sequence(wychyly_serw_up_down, "🕷️ podnoszenie/opuszczanie", step_duration=0.05)
        time.sleep(1.0)
        
        # 2. Crouch and rise like preparing for action
        self.get_logger().info('🕷️ --- Faza 2: Kucanie i wstawanie ---')
        self.execute_sequence(wychyly_serw_crouch_rise, "🕷️ kucanie-wstawanie", step_duration=0.06)
        time.sleep(1.0)

        # 6. Tilting movements - forward/back
        self.get_logger().info('🕷️ --- Faza 6: Pochylanie przód-tył ---')
        self.execute_sequence(wychyly_serw_tilt_forward_back, "🕷️ pochylanie przód-tył", step_duration=0.05)
        time.sleep(1.0)
        
        # 7. Tilting movements - left/right
        self.get_logger().info('🕷️ --- Faza 7: Pochylanie lewo-prawo ---')
        self.execute_sequence(wychyly_serw_tilt_left_right, "🕷️ pochylanie lewo-prawo", step_duration=0.05)
        time.sleep(1.0)
        
        # 8. Figure 8 movement
        self.get_logger().info('🕷️ --- Faza 8: Ruch ósemkowy ---')
        self.execute_sequence(wychyly_serw_figure8, "🕷️ ruch ósemkowy", step_duration=0.06)
        time.sleep(1.0)
        

        # 10. Zigzag movement
        self.get_logger().info('🕷️ --- Faza 10: Ruch zigzag ---')
        self.execute_sequence(wychyly_serw_zigzag, "🕷️ ruch zigzag", step_duration=0.05)
        time.sleep(1.0)
        
        # 11. Wave motion through body
        self.get_logger().info('🕷️ --- Faza 11: Ruch falowy ---')
        self.execute_sequence(wychyly_serw_wave_motion, "🕷️ ruch falowy", step_duration=0.05)
        time.sleep(1.0)
        
        # 12. End with shake movement
        self.get_logger().info('🕷️ --- Faza 12 (finał): Potrząsanie ---')
        self.execute_sequence(wychyly_serw_shake, "🕷️ potrząsanie", step_duration=0.04)
        time.sleep(1.0)
        
        # 13. Final up-down to settle
        self.get_logger().info('🕷️ --- Faza finalna: Spokojne osiadanie ---')
        self.execute_sequence(wychyly_serw_up_down, "🕷️ finalne osiadanie", step_duration=0.08)
        
        self.get_logger().info('🕷️ ===== KOMPLETNY TANIEC PAJĄCZKA ZAKOŃCZONY! =====')

def main(args=None):
    rclpy.init(args=args)
    
    # Utworzenie węzła pajączka z debugowaniem
    node = SpiderLegSequencePlayer(debug_joints=True)
    
    try:
        # Krótkie oczekiwanie na inicjalizację
        print("🕷️ Inicjalizacja pajączka... Poczekaj 3 sekundy.")
        time.sleep(3.0)
        
        # Execute complete spider dance sequence
        node.execute_complete_spider_dance_sequence()
        
        # Utrzymanie węzła aktywnego przez chwilę
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        print("\n🕷️ Pajączek przerwany przez użytkownika")
    
    # Sprzątanie
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()