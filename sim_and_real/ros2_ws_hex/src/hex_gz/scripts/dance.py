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

# Updated from bi_gate - using znajdz_punkty_kwadratowe instead of znajdz_punkty_rowno_odlegle_na_paraboli
def znajdz_punkty_kwadratowe(r, h, ilosc_punktow_na_krzywej, ilosc_probek, bufor_y):
    """
    Generuje punkty dla ruchu kwadratowego: w górę -> do przodu -> w dół
    r - zasięg ruchu w kierunku Y
    h - wysokość podniesienia
    ilosc_punktow_na_krzywej - liczba punktów na całej trajektorii
    ilosc_probek - nie używane (zachowane dla kompatybilności)
    bufor_y - przesunięcie w kierunku Y
    """
    punkty = []
    
    # Podział punktów na 3 fazy: w górę, do przodu, w dół
    punkty_w_gore = max(1, ilosc_punktow_na_krzywej // 4)  # 25% punktów na ruch w górę
    punkty_do_przodu = max(1, ilosc_punktow_na_krzywej // 2)  # 50% punktów na ruch do przodu
    punkty_w_dol = ilosc_punktow_na_krzywej - punkty_w_gore - punkty_do_przodu  # reszta na ruch w dół
    
    # Faza 1: Ruch w górę (Z zwiększa się, Y stałe)
    for i in range(punkty_w_gore):
        z_val = (i + 1) * h / punkty_w_gore
        punkty.append([0, bufor_y, z_val])
    
    # Faza 2: Ruch do przodu (Z stałe na wysokości h, Y zwiększa się)
    for i in range(punkty_do_przodu):
        y_val = bufor_y + (i + 1) * r / punkty_do_przodu
        punkty.append([0, y_val, h])
    
    # Faza 3: Ruch w dół (Z maleje, Y stałe)
    for i in range(punkty_w_dol):
        z_val = h - (i + 1) * h / punkty_w_dol
        punkty.append([0, bufor_y + r, z_val])
    
    return punkty

# Updated from bi_gate - corrected formula
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
        required_r = target_distance / (1 + 2 * cycles)  # Corrected formula from bi_gate
        
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
    Generuje trajektorię chodu dla zadanej odległości - updated from bi_gate
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

    # Updated to use znajdz_punkty_kwadratowe from bi_gate
    punkty_etap1_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, 10000, 0)
    punkty_etap2_ruchu_y = np.linspace(r * (ilosc_punktow_na_krzywych - 1) / ilosc_punktow_na_krzywych, 0, ilosc_punktow_na_krzywych)
    punkty_etap2_ruchu = [[0, punkty_etap2_ruchu_y[i], 0] for i in range(ilosc_punktow_na_krzywych)]
    punkty_etap3_ruchu_y = np.linspace(-r / ilosc_punktow_na_krzywych, -r, ilosc_punktow_na_krzywych)
    punkty_etap3_ruchu = [[0, punkty_etap3_ruchu_y[i], 0] for i in range(ilosc_punktow_na_krzywych)]
    punkty_etap4_ruchu = znajdz_punkty_kwadratowe(2 * r, h, ilosc_punktow_na_krzywych, 20000, -r)
    punkty_etap5_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, 10000, -r)

    # Build cycles as in bi_gate
    zera = []
    for i in range(ilosc_punktow_na_krzywych):
        zera.append([0, 0, 0])

    pierwszy_krok_nóg_1_4 = punkty_etap5_ruchu.copy()
    pierwszy_krok_nóg_1_4.reverse()
    pierwszy_krok_nóg_2_5 = zera.copy()
    pierwszy_krok_nóg_3_6 = zera.copy()

    wypelniacz = []
    for i in range(ilosc_punktow_na_krzywych):
        wypelniacz.append(pierwszy_krok_nóg_1_4[ilosc_punktow_na_krzywych - 1])

    drugi_krok_nóg_1_4 = wypelniacz.copy()
    drugi_krok_nóg_2_5 = zera.copy()
    drugi_krok_nóg_3_6 = punkty_etap1_ruchu.copy()

    cykl_nog_1_4 = pierwszy_krok_nóg_1_4.copy()
    cykl_nog_2_5 = pierwszy_krok_nóg_2_5.copy()
    cykl_nog_3_6 = pierwszy_krok_nóg_3_6.copy()

    cykl_nog_1_4 = np.concatenate([cykl_nog_1_4, drugi_krok_nóg_1_4])
    cykl_nog_2_5 = np.concatenate([cykl_nog_2_5, drugi_krok_nóg_2_5])
    cykl_nog_3_6 = np.concatenate([cykl_nog_3_6, drugi_krok_nóg_3_6])

    # Main loop z obliczoną liczbą cykli
    for _ in range(optimal_cycles):
        cykl_nog_1_4 = np.concatenate([cykl_nog_1_4, punkty_etap4_ruchu, punkty_etap2_ruchu, punkty_etap3_ruchu])
        cykl_nog_2_5 = np.concatenate([cykl_nog_2_5, punkty_etap3_ruchu, punkty_etap4_ruchu, punkty_etap2_ruchu])
        cykl_nog_3_6 = np.concatenate([cykl_nog_3_6, punkty_etap2_ruchu, punkty_etap3_ruchu, punkty_etap4_ruchu])

    # Shutdown
    wypelniacz = []
    for i in range(ilosc_punktow_na_krzywych):
        wypelniacz.append(punkty_etap4_ruchu[ilosc_punktow_na_krzywych - 1])

    cykl_nog_1_4 = np.concatenate([cykl_nog_1_4, punkty_etap5_ruchu, zera])
    cykl_nog_2_5 = np.concatenate([cykl_nog_2_5, zera, zera])
    cykl_nog_3_6 = np.concatenate([cykl_nog_3_6, wypelniacz, punkty_etap1_ruchu.copy()[::-1]])

    return np.array(cykl_nog_1_4), np.array(cykl_nog_2_5), np.array(cykl_nog_3_6)

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

# Generate different movement sequences
print("Generating movement sequences...")

# 1. Walking trajectory - updated to use bi_gate structure
TARGET_DISTANCE = 0.25  # 25 cm
cykl_nog_1_4, cykl_nog_2_5, cykl_nog_3_6 = generate_walking_trajectory(TARGET_DISTANCE, l1, l2, l3)

# Convert walking to servo angles format using bi_gate structure
cykle_nog = np.array([
    [
        [ cykl_nog_1_4[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
          cykl_nog_1_4[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
          cykl_nog_1_4[i][2]]
        for i in range(len(cykl_nog_1_4))
    ] if j in (0,3) else
    [
        [cykl_nog_2_5[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nog_2_5[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nog_2_5[i][2]]
        for i in range(len(cykl_nog_2_5))
    ] if j in (1,4) else
    [
        [cykl_nog_3_6[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nog_3_6[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nog_3_6[i][2]]
        for i in range(len(cykl_nog_3_6))
    ]
    for j in range(6)
])

polozenia_stop_podczas_cyklu = np.array([ # polozenie_stop jest wzgledem ukladu nogi, gdzie przyczep do tulowia to punkt 0,0,0
    [[
        stopa_spoczynkowa[0] + cykle_nog[j][i][0],
        stopa_spoczynkowa[1] + cykle_nog[j][i][1],
        stopa_spoczynkowa[2] + cykle_nog[j][i][2]
    ]
    for i in range(len(cykl_nog_1_4))]
    for j in range(6)
])

#wychyly podawane odpowiednio dla 1 2 i 3 przegubu w radianach
wychyly_serw_podczas_ruchu_walking = np.array([
[katy_serw(polozenia_stop_podczas_cyklu[j][i], l1, l2, l3)
    for i in range(len(cykl_nog_1_4))]
    for j in range(6)
])

print("Generating body movement sequences...")
wychyly_serw_podczas_ruchu_oscillation = generate_body_movement_servo_angles('oscillation', duration=3.0, amplitude=0.03)
wychyly_serw_podczas_ruchu_circle = generate_body_movement_servo_angles('circle', duration=4.0, amplitude=0.025)
wychyly_serw_podczas_ruchu_figure8 = generate_body_movement_servo_angles('figure8', duration=3.0, amplitude=0.02)
wychyly_serw_podczas_ruchu_custom = generate_body_movement_servo_angles('custom', duration=2.0, amplitude=0.04)

print("All movement sequences generated!")

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

class LegSequencePlayer(Node):
    def __init__(self, debug_joints=True):
        super().__init__('leg_sequence_player')
        self.get_logger().info('Inicjalizacja węzła do sekwencji ruchów z ruchem platformy')
        
        # Debug flag for joint position printing
        self.debug_joints = debug_joints
        
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

    def print_foot_positions(self, wychyly_serw_array, step_index, sequence_name=""):
        """Print calculated foot positions for debugging"""
        if not self.debug_joints:
            return
            
        print(f"\n📍 POZYCJE STOP - Krok {step_index} ({sequence_name})")
        
        for leg_num in range(6):
            joint_values = wychyly_serw_array[leg_num][step_index]
            alfa_1, alfa_2, alfa_3 = joint_values
            
            # Calculate foot position using forward kinematics
            P0 = np.array([0, 0, 0])  # Joint base
            P1 = P0 + np.array([l1 * np.cos(alfa_1), l1 * np.sin(alfa_1), 0])
            P2 = P1 + np.array([np.cos(alfa_1)*np.cos(alfa_2)*l2, np.sin(alfa_1)*np.cos(alfa_2)*l2, np.sin(alfa_2) * l2])
            P3 = P2 + np.array([np.cos(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_2 - alfa_3) * l3])
            
            # Transform to world coordinates
            leg_orientation = leg_orientations[leg_num]
            rotation_matrix = np.array([
                [np.cos(leg_orientation), -np.sin(leg_orientation), 0],
                [np.sin(leg_orientation), np.cos(leg_orientation), 0],
                [0, 0, 1]
            ])
            
            P3_world = rotation_matrix @ P3 + leg_attachments[leg_num]
            
            print(f"Noga {leg_num+1}: leg_frame=({P3[0]:7.4f}, {P3[1]:7.4f}, {P3[2]:7.4f}) world=({P3_world[0]:7.4f}, {P3_world[1]:7.4f}, {P3_world[2]:7.4f})")
        
        print("-" * 50)
        
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

    def send_trajectory_to_all_legs_at_step(self, wychyly_serw_array, step_index, duration_sec=0.1, sequence_name=""):
        """
        Wysyła trajektorię do kontrolerów wszystkich nóg jednocześnie
        używając wartości z tablicy wychyly_serw_array dla danego kroku
        """
        self.get_logger().info(f'Wysyłam trajektorię dla kroku {step_index}')
        
        # Sprawdź, czy indeks jest prawidłowy
        if step_index >= len(wychyly_serw_array[0]):
            self.get_logger().error(f'Indeks kroku {step_index} jest poza zakresem!')
            return False
        
        # Print joint positions for debugging
        self.print_joint_positions(wychyly_serw_array, step_index, sequence_name)
        
        # Also print foot positions
        self.print_foot_positions(wychyly_serw_array, step_index, sequence_name)
        
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
        
        # Check sequence validity first
        self.check_sequence_validity(wychyly_serw_array, sequence_name)
        
        # Jeśli nie podano end_step, użyj całej tablicy
        if end_step is None:
            end_step = len(wychyly_serw_array[0])
        
        # Czekanie na inicjalizację czasu symulacji
        self.get_logger().info('Czekam na czas symulacji...')
        while self.sim_time is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Przejście do pozycji początkowej (pierwszy punkt w tablicy)
        self.send_trajectory_to_all_legs_at_step(wychyly_serw_array, start_step, duration_sec=0.15, sequence_name=sequence_name)
        self.get_logger().info('Oczekiwanie na wykonanie początkowego ruchu...')
        self.wait_sim_time(0.15)
        
        # Wykonanie sekwencji ruchów
        for step in range(start_step + 1, end_step):
            # Print joint positions every step for first 10 steps, then every 10 steps
            if step <= 10 or step % 10 == 0:
                self.send_trajectory_to_all_legs_at_step(wychyly_serw_array, step, duration_sec=step_duration, sequence_name=sequence_name)
            else:
                # Silent mode for intermediate steps
                old_debug = self.debug_joints
                self.debug_joints = False
                self.send_trajectory_to_all_legs_at_step(wychyly_serw_array, step, duration_sec=step_duration, sequence_name=sequence_name)
                self.debug_joints = old_debug
                
            if step % 20 == 0:  # Log every 20 steps
                self.get_logger().info(f'{sequence_name}: krok {step}/{end_step}, oczekiwanie {step_duration}s...')
            self.wait_sim_time(step_duration)
        
        self.get_logger().info(f'Sekwencja {sequence_name} zakończona')

    def execute_complete_dance_sequence(self):
        """Execute complete dance sequence with all movement types"""
        self.get_logger().info('Rozpoczynam sekwencję taneczną z parametrami z bi_gate')
        
        # Pre-check all sequences
        print("\n🔍 SPRAWDZANIE WSZYSTKICH SEKWENCJI PRZED ROZPOCZĘCIEM:")
        self.check_sequence_validity(wychyly_serw_podczas_ruchu_oscillation, "oscylacja platformy")
        self.check_sequence_validity(wychyly_serw_podczas_ruchu_circle, "ruch kołowy platformy")
        self.check_sequence_validity(wychyly_serw_podczas_ruchu_figure8, "ruch ósemkowy platformy")
        self.check_sequence_validity(wychyly_serw_podczas_ruchu_walking, f"chód na {TARGET_DISTANCE}m")
        self.check_sequence_validity(wychyly_serw_podczas_ruchu_custom, "niestandardowy ruch")
        
        # Wait for initialization
        self.get_logger().info('Czekam na inicjalizację...')
        self.wait_sim_time(2.0)
        
        # 1. Body oscillation movement
        self.get_logger().info('--- Faza 1: Ruch oscylacyjny platformy ---')
        self.execute_sequence(wychyly_serw_podczas_ruchu_oscillation, "oscylacja platformy", step_duration=0.05)
        self.wait_sim_time(1.0)



def main(args=None):
    rclpy.init(args=args)
    
    # Utworzenie węzła z debugowaniem
    node = LegSequencePlayer(debug_joints=True)
    
    try:
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