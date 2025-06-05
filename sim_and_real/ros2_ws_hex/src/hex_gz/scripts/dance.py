#!/usr/bin/env python3
#code to move platform without moving foots

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rosgraph_msgs.msg import Clock
import time
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import matplotlib.animation as animation

matplotlib.use('TkAgg')

# Joint limits for servo motors
JOINT_LIMITS = {
    'alfa_1': (-np.pi/4, np.pi/4),       # coxa joint
    'alfa_2': (-np.pi/3, np.pi/3),       # femur joint  
    'alfa_3': (-np.pi/2, np.pi/6)        # tibia joint
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
            # If IK fails, use safe default angles within limits
            default_angles = [0, 0, np.radians(-30)]
            # Apply limits to default angles too
            safe_angles = apply_joint_limits(*default_angles)
            joint_angles.append(list(safe_angles))
    
    return np.array(joint_angles)

def funkcja_ruchu_nogi(r, h, y_punktu): #y_punktu jest w ukladzie wspolrzednych srodka robota
    return (-4 * h * (y_punktu ** 2)) / (r ** 2) + (4 * h * y_punktu) / r

def dlugosc_funkcji_ruchu_nogi(r, h, ilosc_probek): #funkcja liczy długosc funkcji na przedziale miedzy miescami zerowymi
    suma = 0
    for i in range(1,ilosc_probek):
        y_0 = funkcja_ruchu_nogi(r, h, (i-1)/ilosc_probek * r)
        y_1 = funkcja_ruchu_nogi(r, h, i/ilosc_probek * r)
        dlugosc = np.sqrt((y_1 - y_0) ** 2 + (r/ilosc_probek) ** 2)
        suma += dlugosc
    return suma

def znajdz_punkty_rowno_odlegle_na_paraboli(r, h, ilosc_punktow_na_krzywej, ilosc_probek, bufor_y):
    L = dlugosc_funkcji_ruchu_nogi(r, h, ilosc_probek)
    dlugosc_kroku = L/ilosc_punktow_na_krzywej
    suma = 0
    punkty = []
    for i in range(1,ilosc_probek):
        z_0 = funkcja_ruchu_nogi(r, h, (i-1)/ilosc_probek * r)
        z_1 = funkcja_ruchu_nogi(r, h, i/ilosc_probek * r)
        dlugosc = np.sqrt((z_1 - z_0) ** 2 + (r/ilosc_probek) ** 2)
        suma += dlugosc
        if(suma > dlugosc_kroku):
            suma = suma - dlugosc_kroku
            punkty.append([0, i/ilosc_probek * r + bufor_y, z_1])
        if(len(punkty) == ilosc_punktow_na_krzywej - 1):
            break
    punkty.append([0, bufor_y + r, 0])
    return punkty

def calculate_optimal_r_and_cycles(target_distance, l3):
    """
    Oblicza optymalne r i liczbę cykli dla danej odległości
    target_distance = r (startup+shutdown) + cycles * 2r (main_loop)
    target_distance = r * (1 + 2*cycles)
    """
    r_max = l3 / 3  # maksymalne r (obecna wartość)
    r_min = l3 / 100  # minimalne r
    
    best_r = None
    best_cycles = None
    
    # Sprawdzaj od największych wartości r w dół
    for cycles in range(1, 1000):
        required_r = target_distance / (1 + 2 * cycles)
        
        if r_min <= required_r <= r_max:
            if best_r is None or required_r > best_r:
                best_r = required_r
                best_cycles = cycles
                
        # Jeśli r stało się za małe, przerwij
        if required_r < r_min:
            break
    
    return best_r, best_cycles

def generate_walking_trajectory(target_distance, l1, l2, l3):
    """
    Generuje trajektorię chodu dla zadanej odległości
    """
    # Oblicz optymalne r i liczbę cykli
    optimal_r, optimal_cycles = calculate_optimal_r_and_cycles(target_distance, l3)
    
    if optimal_r is None:
        raise ValueError(f"Nie można wygenerować trajektorii dla odległości {target_distance}m")
    
    print(f"Obliczone parametry:")
    print(f"  Docelowa odległość: {target_distance}m")
    print(f"  Optymalne r: {optimal_r:.4f}m")
    print(f"  Liczba cykli main_loop: {optimal_cycles}")
    print(f"  Rzeczywista odległość: {optimal_r * (1 + 2 * optimal_cycles):.4f}m")
    
    # Używaj optimal_r zamiast stałego r
    r = optimal_r
    h = l3 / 4  # wysokość pozostaje stała
    ilosc_punktow_na_krzywych = 10

    punkty_etap1_ruchu = znajdz_punkty_rowno_odlegle_na_paraboli(r, h / 2, ilosc_punktow_na_krzywych, 10000, 0)
    punkty_etap2_ruchu_y = np.linspace(r * (ilosc_punktow_na_krzywych - 1) / ilosc_punktow_na_krzywych, 0, ilosc_punktow_na_krzywych)
    punkty_etap2_ruchu = [[0, punkty_etap2_ruchu_y[i], 0] for i in range(ilosc_punktow_na_krzywych)]
    punkty_etap3_ruchu_y = np.linspace(-r / ilosc_punktow_na_krzywych, -r, ilosc_punktow_na_krzywych)
    punkty_etap3_ruchu = [[0, punkty_etap3_ruchu_y[i], 0] for i in range(ilosc_punktow_na_krzywych)]
    punkty_etap4_ruchu = znajdz_punkty_rowno_odlegle_na_paraboli(2 * r, h, 2 * ilosc_punktow_na_krzywych, 20000, -r)
    punkty_etap5_ruchu = znajdz_punkty_rowno_odlegle_na_paraboli(r, h / 2, ilosc_punktow_na_krzywych, 10000, -r)

    # Startup
    cykl_ogolny_nog_1_3_5 = punkty_etap1_ruchu.copy()
    cykl_ogolny_nog_2_4_6 = punkty_etap3_ruchu.copy()

    # Main loop z obliczoną liczbą cykli
    for _ in range(optimal_cycles):
        cykl_ogolny_nog_1_3_5 += punkty_etap2_ruchu + punkty_etap3_ruchu + punkty_etap4_ruchu
        cykl_ogolny_nog_2_4_6 += punkty_etap4_ruchu + punkty_etap2_ruchu + punkty_etap3_ruchu

    # Shutdown
    cykl_ogolny_nog_1_3_5 += punkty_etap2_ruchu + punkty_etap3_ruchu + punkty_etap5_ruchu
    cykl_ogolny_nog_2_4_6 += punkty_etap4_ruchu + punkty_etap2_ruchu

    cykl_ogolny_nog_1_3_5 = np.array(cykl_ogolny_nog_1_3_5)
    cykl_ogolny_nog_2_4_6 = np.array(cykl_ogolny_nog_2_4_6)

    return np.array(cykl_ogolny_nog_1_3_5), np.array(cykl_ogolny_nog_2_4_6)

def generate_body_movement_sequence(movement_type='oscillation', duration=5.0, amplitude=0.05):
    """
    Generate body movement sequence keeping feet fixed
    movement_type: 'oscillation', 'circle', 'figure8', 'custom'
    """
    num_frames = int(duration * 20)  # 20 Hz
    t = np.linspace(0, duration, num_frames)
    
    body_poses = []
    
    if movement_type == 'oscillation':
        for i in range(num_frames):
            x = amplitude * np.sin(2 * np.pi * t[i] * 0.5)  # 0.5 Hz frequency
            y = amplitude * 0.5 * np.sin(2 * np.pi * t[i] * 0.3)
            z = amplitude * 0.3 * np.sin(2 * np.pi * t[i]) + 0.02
            roll = amplitude * 2 * np.sin(2 * np.pi * t[i] * 0.7)
            pitch = amplitude * 3 * np.sin(2 * np.pi * t[i] * 0.5)
            yaw = amplitude * 4 * np.sin(2 * np.pi * t[i] * 0.3)
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    elif movement_type == 'circle':
        radius = amplitude
        for i in range(num_frames):
            angle = 2 * np.pi * t[i] / duration
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = amplitude * 0.3 * np.sin(4 * np.pi * t[i] / duration) + 0.02
            roll = amplitude * np.sin(angle)
            pitch = amplitude * np.cos(angle)
            yaw = angle * 0.5
            body_poses.append([x, y, z, roll, pitch, yaw])
            
    elif movement_type == 'figure8':
        for i in range(num_frames):
            angle = 2 * np.pi * t[i] / duration
            x = amplitude * np.sin(angle)
            y = amplitude * np.sin(angle) * np.cos(angle)
            z = amplitude * 0.2 * np.sin(2 * angle) + 0.02
            roll = amplitude * np.sin(2 * angle)
            pitch = amplitude * np.cos(angle)
            yaw = amplitude * 0.5 * np.sin(angle)
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    else:  # custom - simple up-down with rotation
        for i in range(num_frames):
            x = 0
            y = 0
            z = amplitude * np.sin(2 * np.pi * t[i] * 0.5) + 0.02
            roll = 0
            pitch = 0
            yaw = amplitude * 2 * np.sin(2 * np.pi * t[i] * 0.2)
            body_poses.append([x, y, z, roll, pitch, yaw])
    
    return np.array(body_poses)

def generate_body_movement_servo_angles(movement_type='oscillation', duration=5.0, amplitude=0.05):
    """
    Generate body movement as servo angles array compatible with the existing ROS system
    Returns wychyly_serw_podczas_ruchu format for body movements
    """
    # Generate body poses
    body_poses = generate_body_movement_sequence(movement_type, duration, amplitude)
    
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

# Długosci segmentow nog - ZAKTUALIZOWANE z rotation.py
l1 = 0.17995 - 0.12184
l2 = 0.30075 - 0.17995
l3 = 0.50975 - 0.30075

# Leg attachment points on hexapod body (relative to body center)
leg_attachments = np.array([
    [ 0.073922, 0.055095 , 0.003148],
    [ 0.0978,  -0.00545,   0.003148],
    [ 0.067301, -0.063754, 0.003148],
    [-0.067301, -0.063754, 0.003148],
    [-0.0978,   -0.00545,  0.003148],
    [-0.073922,  0.055095, 0.003148],
])

# Leg orientations relative to body
leg_orientations = np.array([
    np.deg2rad(37.169), 0, np.deg2rad(-37.169), 
    np.deg2rad(180 + 37.169), np.deg2rad(180), np.deg2rad(180 - 37.169)
])

# Położenie punktu spoczynku od przyczepu nogi wyznaczone na bazie katow przgubow podczas spoczynku
# WAZNE !!! jest to polozenie stopy w ukladzie punktu zaczepienia stopy a nie ukladu XYZ
# w ktorym X1 to prostopadła prosta do boku platformy do ktorej noga jest zaczepiona i rosnie w kierunku od hexapoda
# Y1 to os pokrywajaca sie z bokiem platformy do ktorego jest przyczepiona noga i rosnie w kierunku przodu hexapoda
# Z1 pokrywa sie z osia Z ukladu XYZ

# zalozone katy spoczynkowe przegubow
alfa_1 = 0
alfa_2 = np.radians(0)
alfa_3 = np.radians(80)

P0 = np.array([0, 0, 0])
P1 = P0 + np.array([l1 * np.cos(alfa_1), l1 *np.sin(alfa_1), 0])
P2 = P1 + np.array([np.cos(alfa_1)*np.cos(alfa_2)*l2,np.sin(alfa_1)*np.cos(alfa_2)*l2, np.sin(alfa_2) * l2])
P3 = P2 + np.array([np.cos(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_2 - alfa_3) * l3])

stopa_spoczynkowa = P3
wysokosc_start = -stopa_spoczynkowa[2]

przyczepy_nog_do_tulowia = leg_attachments
nachylenia_nog_do_bokow_platformy_pajaka = leg_orientations

# Calculate fixed foot positions in world coordinates for body movement
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

# Generate different movement sequences
print("Generating movement sequences...")

# 1. Walking trajectory
TARGET_DISTANCE = 0.25  # 25 cm
cykl_ogolny_nog_1_3_5, cykl_ogolny_nog_2_4_6 = generate_walking_trajectory(TARGET_DISTANCE, l1, l2, l3)

# Convert walking to servo angles format (jak w leg_sequence_player.py)
cykle_nog = np.array([
    [
        [cykl_ogolny_nog_1_3_5[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_ogolny_nog_1_3_5[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_ogolny_nog_1_3_5[i][2]]
        for i in range(len(cykl_ogolny_nog_1_3_5))
    ] if j in (0, 2, 4) else
    [
        [cykl_ogolny_nog_2_4_6[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_ogolny_nog_2_4_6[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_ogolny_nog_2_4_6[i][2]]
        for i in range(len(cykl_ogolny_nog_2_4_6))
    ]
    for j in range(6)
])

polozenia_stop_podczas_cyklu = np.array([ # polozenie_stop jest wzgledem ukladu nogi, gdzie przyczep do tulowia to punkt 0,0,0
    [[
        stopa_spoczynkowa[0] + cykle_nog[j][i][0],
        stopa_spoczynkowa[1] + cykle_nog[j][i][1],
        stopa_spoczynkowa[2] + cykle_nog[j][i][2]
    ]
    for i in range(len(cykl_ogolny_nog_1_3_5))]
    for j in range(6)
])

#wychyly podawane odpowiednio dla 1 2 i 3 przegubu w radianach
wychyly_serw_podczas_ruchu_walking = np.array([
[katy_serw(polozenia_stop_podczas_cyklu[j][i], l1, l2, l3)
    for i in range(len(cykl_ogolny_nog_1_3_5))]
    for j in range(6)
])

print("Generating body movement sequences...")
wychyly_serw_podczas_ruchu_oscillation = generate_body_movement_servo_angles('oscillation', duration=3.0, amplitude=0.03)
#wychyly_serw_podczas_ruchu_circle = generate_body_movement_servo_angles('circle', duration=4.0, amplitude=0.025)
#wychyly_serw_podczas_ruchu_figure8 = generate_body_movement_servo_angles('figure8', duration=3.0, amplitude=0.02)
#wychyly_serw_podczas_ruchu_custom = generate_body_movement_servo_angles('custom', duration=2.0, amplitude=0.04)

print("All movement sequences generated!")

class LegSequencePlayer(Node):
    def __init__(self):
        super().__init__('leg_sequence_player')
        self.get_logger().info('Inicjalizacja węzła do sekwencji ruchów z ruchem platformy')
        
        # Subskrypcja do czasu symulacji
        self.clock_subscriber = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        
        self.sim_time = None
        self.last_sim_time = None
        
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

    def clock_callback(self, msg):
        """Callback do odbioru czasu symulacji"""
        self.last_sim_time = self.sim_time
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def wait_sim_time(self, duration_sec):
        """Czeka określony czas w czasie symulacji"""
        if self.sim_time is None:
            self.get_logger().warn('Brak czasu symulacji, używam time.sleep')
            time.sleep(duration_sec)
            return
            
        start_time = self.sim_time
        target_time = start_time + duration_sec
        
        while self.sim_time < target_time:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.sim_time is None:
                break

    def send_trajectory_to_all_legs_at_step(self, wychyly_serw_array, step_index, duration_sec=0.1):
        """
        Wysyła trajektorię do kontrolerów wszystkich nóg jednocześnie
        używając wartości z tablicy wychyly_serw_array dla danego kroku
        """
        self.get_logger().info(f'Wysyłam trajektorię dla kroku {step_index}')
        
        # Sprawdź, czy indeks jest prawidłowy
        if step_index >= len(wychyly_serw_array[0]):
            self.get_logger().error(f'Indeks kroku {step_index} jest poza zakresem!')
            return False
        
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
        
        self.get_logger().info('Wysłano trajektorie dla wszystkich nóg')
        return True
    
    def execute_sequence(self, wychyly_serw_array, sequence_name="sequence", start_step=0, end_step=None, step_duration=0.02):
        """
        Wykonanie sekwencji ruchów dla wszystkich nóg równocześnie
        używając wartości z tablicy wychyly_serw_array
        """
        self.get_logger().info(f'Rozpoczynam sekwencję: {sequence_name}')
        
        # Jeśli nie podano end_step, użyj całej tablicy
        if end_step is None:
            end_step = len(wychyly_serw_array[0])
        
        # Czekanie na inicjalizację czasu symulacji
        self.get_logger().info('Czekam na czas symulacji...')
        while self.sim_time is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Przejście do pozycji początkowej (pierwszy punkt w tablicy)
        self.send_trajectory_to_all_legs_at_step(wychyly_serw_array, start_step, duration_sec=0.15)
        self.get_logger().info('Oczekiwanie na wykonanie początkowego ruchu...')
        self.wait_sim_time(0.15)
        
        # Wykonanie sekwencji ruchów
        for step in range(start_step + 1, end_step):
            self.send_trajectory_to_all_legs_at_step(wychyly_serw_array, step, duration_sec=step_duration)
            if step % 20 == 0:  # Log every 20 steps
                self.get_logger().info(f'{sequence_name}: krok {step}/{end_step}, oczekiwanie {step_duration}s...')
            self.wait_sim_time(step_duration)
        
        self.get_logger().info(f'Sekwencja {sequence_name} zakończona')

    def execute_complete_dance_sequence(self):
        """Execute complete dance sequence with all movement types"""
        self.get_logger().info('nie tance byly celem, ale niech tak bedzie')
        
        # Wait for initialization
        self.get_logger().info('Czekam na inicjalizację...')
        self.wait_sim_time(2.0)
        
        # 1. Body oscillation movement
        self.get_logger().info('--- Faza 1: Ruch oscylacyjny platformy ---')
        self.execute_sequence(wychyly_serw_podczas_ruchu_oscillation, "oscylacja platformy", step_duration=0.05)
        self.wait_sim_time(1.0)
        
        # 2. Body circular movement
        self.get_logger().info('--- Faza 2: Ruch kołowy platformy ---')
        self.execute_sequence(wychyly_serw_podczas_ruchu_circle, "ruch kołowy platformy", step_duration=0.05)
        self.wait_sim_time(1.0)
        
        # 3. Body figure-8 movement
        self.get_logger().info('--- Faza 3: Ruch ósemkowy platformy ---')
        self.execute_sequence(wychyly_serw_podczas_ruchu_figure8, "ruch ósemkowy platformy", step_duration=0.05)
        self.wait_sim_time(1.0)
        
        # 4. Traditional walking for comparison
        self.get_logger().info('--- Faza 4: Tradycyjny chód (dla porównania) ---')
        self.execute_sequence(wychyly_serw_podczas_ruchu_walking, f"chód na {TARGET_DISTANCE}m", step_duration=0.02)
        self.wait_sim_time(1.0)
        
        # 5. Custom body movement
        self.get_logger().info('--- Faza 5: Niestandardowy ruch platformy ---')
        self.execute_sequence(wychyly_serw_podczas_ruchu_custom, "niestandardowy ruch", step_duration=0.05)
        
        self.get_logger().info('=== SEKWENCJA TANECZNA ZAKOŃCZONA ===')


def main(args=None):
    rclpy.init(args=args)
    
    # Utworzenie węzła
    node = LegSequencePlayer()
    
    try:
        print("=== ENHANCED HEXAPOD CONTROLLER WITH BODY MOVEMENT ===")
        print("Features:")
        print("- Body oscillation movement (keeping feet fixed)")
        print("- Body circular movement") 
        print("- Body figure-8 movement")
        print("- Traditional walking movement")
        print("- Custom body movement")
        print("- Uses same ROS topics mechanism as colleague's code")
        print("- Joint limits enforcement")
        print("=========================================================")
        
        # Krótkie oczekiwanie na inicjalizację
        print("Inicjalizacja... Poczekaj 2 sekundy.")
        node.wait_sim_time(2.0)
        
        # Execute complete dance sequence
        node.execute_complete_dance_sequence()
        
        # Utrzymanie węzła aktywnego przez chwilę
        node.wait_sim_time(1.0)
        
    except KeyboardInterrupt:
        print("\nPrzerwano przez użytkownika")
    
    # Sprzątanie
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()