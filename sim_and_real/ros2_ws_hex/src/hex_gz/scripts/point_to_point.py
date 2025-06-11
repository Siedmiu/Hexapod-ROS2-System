#!/usr/bin/env python3
"""

This program implements the hexapod's point-to-point walk
To add a point for the hexapod to walk to, add self.move_to_point(x, y) in the execute_complete_sequence() function.
The program determines the angle and distance from the current point to the target point, and then executes the rotation of the robot by the given angle and, in turn, the movement by the given distance
The movement is performed using a tri-gate

"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rosgraph_msgs.msg import Clock
import time
import sys


matplotlib.use('TkAgg')

def katy_serw(P3, l1, l2, l3):
    # inverse kinematics
    alfa_1 = np.arctan2(P3[1], P3[0])

    P1 = np.array([l1 * np.cos(alfa_1), l1 * np.sin(alfa_1), 0])

    d = np.sqrt((P3[0] - P1[0]) ** 2 + (P3[1] - P1[1]) ** 2 + (P3[2] - P1[2]) ** 2)

    cos_fi = (l2 ** 2 + l3 ** 2 - d ** 2) / (2 * l2 * l3)
    cos_fi = np.clip(cos_fi, -1.0, 1.0)
    fi = np.arccos(cos_fi)
    alfa_3 = np.deg2rad(180) - fi

    epsilon = np.arcsin(np.sin(fi) * l3 / d)
    tau = np.arctan2(P3[2] - P1[2], np.sqrt((P3[0] - P1[0]) ** 2 + (P3[1] - P1[1]) ** 2))

    alfa_2 = -(epsilon + tau)
    return [alfa_1, alfa_2, alfa_3]

def trajektoria_prostokatna(start, cel, h, liczba_punktow):
    #square trajectory
    liczba_punktow += 3
    start_gora = start + np.array([0, 0, h])
    cel_gora = cel + np.array([0, 0, h])

    etap1 = np.linspace(start, start_gora, liczba_punktow // 3)
    etap2 = np.linspace(start_gora, cel_gora, liczba_punktow // 3)
    etap3 = np.linspace(cel_gora, cel, liczba_punktow - len(etap1) - len(etap2))

    punkty = np.concatenate([etap1[1:], etap2[1:], etap3[1:]], axis=0)
    return punkty

l1 = 0.17995 - 0.12184
l2 = 0.30075 - 0.17995
l3 = 0.50975 - 0.30075


def turn_hexapod(R, alfa, x_start, z):
    x_new = (x_start + R) * np.cos(alfa) - R
    y_new = (x_start + R) * np.sin(alfa)
    return np.array([x_new, y_new, z])

def generate_rotation_sequence(kat_calkowity_deg):
    """Generation of rotation sequences - ORIGINAL LOGIC from basic gates"""
    
    alfa_1 = 0
    alfa_2 = np.deg2rad(10)
    alfa_3 = np.deg2rad(80)

    kat_calkowity = np.radians(kat_calkowity_deg)

    odleglosc_przegubow_od_srodka_hexapoda = 0.1218
    kat_obrotu_cyklu = np.radians(20)
    kat_obrotu = kat_obrotu_cyklu / 2
    
    kierunek = np.sign(kat_calkowity)
    kat_obrotu *= kierunek

    #warunkiem kręcenia jest alfa1 = 0 
    x_start = l1 + l2 * np.cos(alfa_2) + l3 * np.sin(np.deg2rad(90) - alfa_2 - alfa_3)
    z_start = -(l2*np.sin(alfa_2) + l3 * np.cos(np.deg2rad(90) - alfa_2 - alfa_3)) 
    h = 0.1 
    ilosc_punktow_na_etap = 30

    punkt_start_dla_kazdej_nogi = [x_start, 0, z_start]
    punkt_P1 = turn_hexapod(odleglosc_przegubow_od_srodka_hexapoda, kat_obrotu, x_start, z_start)
    punkt_P2 = turn_hexapod(odleglosc_przegubow_od_srodka_hexapoda, -kat_obrotu, x_start, z_start)
    punkt_szczytowy_etapu_1 = (punkt_start_dla_kazdej_nogi + punkt_P1) / 2 + np.array([0, 0, h])
    punkt_szczytowy_etapu_5 = (punkt_start_dla_kazdej_nogi + punkt_P2) / 2 + np.array([0, 0, h])

    etap_1 = np.array(trajektoria_prostokatna(punkt_start_dla_kazdej_nogi, punkt_P1, h, ilosc_punktow_na_etap))
    etap_2 = np.linspace(punkt_P1, punkt_start_dla_kazdej_nogi, ilosc_punktow_na_etap)[1:]
    etap_3 = np.linspace(punkt_start_dla_kazdej_nogi, punkt_P2, ilosc_punktow_na_etap)[1:]
    etap_5 = np.array(trajektoria_prostokatna(punkt_P2, punkt_start_dla_kazdej_nogi, h, ilosc_punktow_na_etap))

    ilosc_cykli = int(np.abs(kat_calkowity) // kat_obrotu_cyklu)

    pozostaly_kat = (np.abs(kat_calkowity) % kat_obrotu_cyklu) / 2 * kierunek

    cykl_nog_1_3_5 = np.concatenate([etap_1, etap_2])
    cykl_nog_2_4_6 = np.concatenate([etap_3, etap_5])

    for _ in range(ilosc_cykli - 1):
        cykl_nog_1_3_5 = np.concatenate([cykl_nog_1_3_5, etap_1, etap_2])
        cykl_nog_2_4_6 = np.concatenate([cykl_nog_2_4_6, etap_3, etap_5])

    if np.abs(pozostaly_kat) > np.radians(1):
        punkt_P1 = turn_hexapod(odleglosc_przegubow_od_srodka_hexapoda, pozostaly_kat, x_start, z_start)
        punkt_P2 = turn_hexapod(odleglosc_przegubow_od_srodka_hexapoda, -pozostaly_kat, x_start, z_start)
        punkt_szczytowy_etapu_1 = (punkt_start_dla_kazdej_nogi + punkt_P1) / 2 + np.array([0, 0, h])
        punkt_szczytowy_etapu_5 = (punkt_start_dla_kazdej_nogi + punkt_P2) / 2 + np.array([0, 0, h])

        etap_1 = np.array(trajektoria_prostokatna(punkt_start_dla_kazdej_nogi, punkt_P1, h, ilosc_punktow_na_etap))
        etap_2 = np.linspace(punkt_P1, punkt_start_dla_kazdej_nogi, ilosc_punktow_na_etap)[1:]
        etap_3 = np.linspace(punkt_start_dla_kazdej_nogi, punkt_P2, ilosc_punktow_na_etap)[1:]
        etap_5 = np.array(trajektoria_prostokatna(punkt_P2, punkt_start_dla_kazdej_nogi, h, ilosc_punktow_na_etap))

        cykl_nog_1_3_5 = np.concatenate([cykl_nog_1_3_5, etap_1, etap_2])
        cykl_nog_2_4_6 = np.concatenate([cykl_nog_2_4_6, etap_3, etap_5])

    wychyly_serw_1_3_5 = []
    wychyly_serw_2_4_6 = []

    # For each leg
    for punkt in cykl_nog_1_3_5:
        kat_obrotu = katy_serw(punkt, l1, l2, l3)
        wychyly_serw_1_3_5.append(kat_obrotu)

    for punkt in cykl_nog_2_4_6:
        kat_obrotu = katy_serw(punkt, l1, l2, l3)
        wychyly_serw_2_4_6.append(kat_obrotu)

    wychyly_serw_1_3_5 = np.array(wychyly_serw_1_3_5)
    wychyly_serw_2_4_6 = np.array(wychyly_serw_2_4_6)

    return wychyly_serw_1_3_5, wychyly_serw_2_4_6

# ============ FUNKCJE DLA MARSZU ============

def znajdz_punkty_kwadratowe(r, h, ilosc_punktow_na_krzywej, ilosc_probek, bufor_y):
    """
    Generates points for square motion starting in point (0, 0, 0): up -> forward -> down
    r - movement range in Y direction
    h - lift height
    ilosc_punktow_na_krzywej - number of points on the entire trajectory
    ilosc_probek - not used (kept for compatibility)
    bufor_y - Y-axis offset 
    """

    punkty = []
    
    # Divide points into 3 phases: up, forward, down

    punkty_w_gore = max(1, ilosc_punktow_na_krzywej // 4) 
    punkty_do_przodu = max(1, ilosc_punktow_na_krzywej // 2)
    punkty_w_dol = ilosc_punktow_na_krzywej - punkty_w_gore - punkty_do_przodu
    
    # Phase 1: up movement
    for i in range(punkty_w_gore):
        z_val = (i + 1) * h / punkty_w_gore
        punkty.append([0, bufor_y, z_val])
    
    # Phase 2: forward movement
    for i in range(punkty_do_przodu):
        y_val = bufor_y + (i + 1) * r / punkty_do_przodu
        punkty.append([0, y_val, h])
    
    # Phase 3: down movement
    for i in range(punkty_w_dol):
        z_val = h - (i + 1) * h / punkty_w_dol
        punkty.append([0, bufor_y + r, z_val])
    
    return punkty

def calculate_optimal_r_and_cycles(target_distance, l3):
    """
    Calculates optimal r and number of cycles for a given distance
    target_distance = r (startup+shutdown) + cycles * 2r (main_loop)
    target_distance = r * (1 + 2*cycles)
    """
    r_max = l3 / 3  # max r (obecna wartość)
    r_min = l3 / 100  # min r
    
    best_r = None
    best_cycles = None
    
    # going down from max r 
    for cycles in range(1, 1000):
        required_r = target_distance / (2 * cycles)
        
        if r_min <= required_r <= r_max:
            if best_r is None or required_r > best_r:
                best_r = required_r
                best_cycles = cycles
    
        if required_r < r_min:
            break
    
    return best_r, best_cycles

def generate_walking_sequence(zadana_odleglosc):
    """Generate walking sequence with precise distance calculation"""

    optimal_r, optimal_cycles = calculate_optimal_r_and_cycles(zadana_odleglosc, l3)
    
    if optimal_r is None:
        raise ValueError(f"Cannot generate distance for r = {zadana_odleglosc}m")
    
    print(f"  target distance: {zadana_odleglosc}m")
    print(f"  Optimal r: {optimal_r:.4f}m")
    print(f"  Cycles in main_loop: {optimal_cycles}")
    print(f"  Real distance: {optimal_r * (1 + 2 * optimal_cycles):.4f}m")
    
    # leg parameters
    alfa_1 = 0
    alfa_2 = np.radians(10)
    alfa_3 = np.radians(80)

    P0 = np.array([0, 0, 0])
    P1 = P0 + np.array([l1 * np.cos(alfa_1), l1 * np.sin(alfa_1), 0])
    P2 = P1 + np.array([np.cos(alfa_1)*np.cos(alfa_2)*l2, np.sin(alfa_1)*np.cos(alfa_2)*l2, np.sin(alfa_2) * l2])
    P3 = P2 + np.array([np.cos(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_2 - alfa_3) * l3])

    x_start = l1 + l2 * np.cos(alfa_2) + l3 * np.sin(np.deg2rad(90) - alfa_2 - alfa_3)  # starting x
    z_start = -(l2*np.sin(alfa_2) + l3 * np.cos(np.deg2rad(90) - alfa_2 - alfa_3))  # starting z

    stopa_spoczynkowa = [x_start, 0, z_start]

    nachylenia_nog_do_bokow_platformy_pajaka = np.array([
        np.deg2rad(45), 0, np.deg2rad(-45), np.deg2rad(180 + 45), np.deg2rad(180), np.deg2rad(180 - 45)
    ])

    r = optimal_r
    h = 0.1
    ilosc_punktow_na_krzywych = 10

    punkty_etap1_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, 10000, 0)
    punkty_etap2_ruchu_y = np.linspace(r * (ilosc_punktow_na_krzywych - 1) / ilosc_punktow_na_krzywych, 0, ilosc_punktow_na_krzywych)
    punkty_etap2_ruchu = [[0, punkty_etap2_ruchu_y[i], 0] for i in range(ilosc_punktow_na_krzywych)]
    punkty_etap3_ruchu_y = np.linspace(-r / ilosc_punktow_na_krzywych, -r, ilosc_punktow_na_krzywych)
    punkty_etap3_ruchu = [[0, punkty_etap3_ruchu_y[i], 0] for i in range(ilosc_punktow_na_krzywych)]
    punkty_etap4_ruchu = znajdz_punkty_kwadratowe(2 * r, h, 2 * ilosc_punktow_na_krzywych, 20000, -r)
    punkty_etap5_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, 10000, -r)

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

    # Cycle for leg
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

    polozenia_stop_podczas_cyklu = np.array([
        [[
            stopa_spoczynkowa[0] + cykle_nog[j][i][0],
            stopa_spoczynkowa[1] + cykle_nog[j][i][1],
            stopa_spoczynkowa[2] + cykle_nog[j][i][2]
        ]
        for i in range(len(cykl_ogolny_nog_1_3_5))]
        for j in range(6)
    ])

#results given for 1st 2 and 3rd joint in radians respectively
    wychyly_serw_podczas_ruchu = np.array([
        [katy_serw(polozenia_stop_podczas_cyklu[j][i], l1, l2, l3)
         for i in range(len(cykl_ogolny_nog_1_3_5))]
        for j in range(6)
    ])

    return wychyly_serw_podczas_ruchu

# ============ KLASA GŁÓWNA ============
class CombinedHexapodController(Node):
    def __init__(self):
        super().__init__('combined_hexapod_controller')
        self.get_logger().info('Initialising hexapod controler')
        
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

        self.estimated_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.estimated_orientation = {'yaw': 0.0}  # Na razie tylko yaw
        self.position_history = []

    def wait_real_time(self, duration_sec):
        self.get_logger().info(f'Waiting {duration_sec}s in real time')
        time.sleep(duration_sec)

    def send_rotation_trajectory(self, wychyly_1_3_5, wychyly_2_4_6, step_index, duration_sec=0.05):
        """Wysyła trajektorię obrotu"""
        self.get_logger().info(f'Sending trajectory to step {step_index}')

        if step_index >= len(wychyly_1_3_5) or step_index >= len(wychyly_2_4_6):
            self.get_logger().error(f'Index of movement {step_index} out of range!')
            return False

        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        for leg_num in range(1, 7):
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names[leg_num]

            point = JointTrajectoryPoint()

            if leg_num in [1, 3, 5]:
                joint_values = wychyly_1_3_5[step_index]
            elif leg_num in [2, 4, 6]:
                joint_values = wychyly_2_4_6[step_index]
            else:
                self.get_logger().error(f'Invalid number of point: {leg_num}')
                continue

            point.positions = list(map(float, joint_values))
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            point.time_from_start = duration

            trajectory.points.append(point)
            self.trajectory_publishers[leg_num].publish(trajectory)

        return True

    def send_walking_trajectory(self, wychyly_serw_podczas_ruchu, step_index, duration_sec=0.02):
        
        if step_index >= len(wychyly_serw_podczas_ruchu[0]):
            self.get_logger().error(f'Index of movement {step_index} out of range!')
            return False
        
        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        for leg_num in range(1, 7):
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names[leg_num]
            
            point = JointTrajectoryPoint()
            joint_values = wychyly_serw_podczas_ruchu[leg_num-1][step_index]
            
            point.positions = [
                float(joint_values[0]),
                float(joint_values[1]),
                float(joint_values[2])
            ]
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            point.time_from_start = duration
            
            trajectory.points.append(point)
            self.trajectory_publishers[leg_num].publish(trajectory)
        
        return True

    def execute_rotation_sequence(self, wychyly_1_3_5, wychyly_2_4_6, step_duration=0.1):
        
        self.get_logger().info('Starting rotation sequence')
        
        max_steps = min(len(wychyly_1_3_5), len(wychyly_2_4_6))

        self.send_rotation_trajectory(wychyly_1_3_5, wychyly_2_4_6, 0, duration_sec=step_duration)
        self.get_logger().info('Waiting for first step')
        self.wait_real_time(step_duration + 0.05)

        for step in range(1, max_steps):
            self.send_rotation_trajectory(wychyly_1_3_5, wychyly_2_4_6, step, duration_sec=step_duration)
            self.wait_real_time(step_duration + 0.05)

        self.get_logger().info('rotation sequence ended')

    def execute_walking_sequence(self, wychyly_serw_podczas_ruchu, step_duration=0.1):

        self.get_logger().info('Starting march sequence')
        
        self.send_walking_trajectory(wychyly_serw_podczas_ruchu, 0, duration_sec=0.15)
        self.wait_real_time(0.15)
        
        for step in range(1, len(wychyly_serw_podczas_ruchu[0])):
            self.send_walking_trajectory(wychyly_serw_podczas_ruchu, step, duration_sec=step_duration)
            self.wait_real_time(step_duration)

        self.get_logger().info('March sequence ended')

    def get_current_pose(self):
        ''' actual position and rotation'''
        return {
            'position': self.estimated_position.copy(),
            'orientation': self.estimated_orientation.copy(),
            'timestamp': time.time()
        }

    def print_current_pose(self, phase_name=""):
        '''printing current position and rotation'''
        pose = self.get_current_pose()
        pos = pose['position']
        ori = pose['orientation']
        self.get_logger().info(
            f'[{phase_name}] Position: x={pos["x"]:.3f}, y={pos["y"]:.3f}, z={pos["z"]:.3f}, '
            f'Orientation: {np.degrees(ori["yaw"]):.1f}°'
        )

    def update_position_after_rotation(self, rotation_angle_deg):

        self.estimated_orientation['yaw'] += np.radians(rotation_angle_deg)

    def update_position_after_walking(self, distance):

        current_yaw = self.estimated_orientation['yaw']
        dx = distance * np.cos(current_yaw)
        dy = distance * np.sin(current_yaw) 
        
        self.estimated_position['x'] += dx
        self.estimated_position['y'] += dy

    def calculate_movement_to_point(self, target_x, target_y):

        current_x = self.estimated_position['x']
        current_y = self.estimated_position['y']
        current_yaw = self.estimated_orientation['yaw']
        
        dx = target_x - current_x
        dy = target_y - current_y
        distance = np.sqrt(dx**2 + dy**2)
        
        target_angle = np.arctan2(dy, dx)
        
        rotation_needed = target_angle - current_yaw
        
        # Normalise point[-π, π]
        while rotation_needed > np.pi:
            rotation_needed -= 2 * np.pi
        while rotation_needed < -np.pi:
            rotation_needed += 2 * np.pi
        
        self.get_logger().info(f'Target: ({target_x:.3f}, {target_y:.3f})')
        self.get_logger().info(f'Distance to target: {distance:.3f}m')
        self.get_logger().info(f'Rotation needed: {np.degrees(rotation_needed):.1f}°')
        
        return np.degrees(rotation_needed), distance

    def move_to_point(self, target_x, target_y):
        """Robot goes to point"""
        self.get_logger().info(f'=== MOVEMENT TO POINT ({target_x}, {target_y}) ===')
        
        # Oblicz potrzebny ruch
        rotation_deg, distance = self.calculate_movement_to_point(target_x, target_y)
        
        # 1. Obrót (jeśli potrzebny)
        if abs(rotation_deg) > 1.0:  # Tylko jeśli obrót > 1 stopień
            self.get_logger().info(f'Rotating by {rotation_deg:.1f}°')
            wychyly_rot_1_3_5, wychyly_rot_2_4_6 = generate_rotation_sequence(rotation_deg)
            self.execute_rotation_sequence(wychyly_rot_1_3_5, wychyly_rot_2_4_6)
            self.update_position_after_rotation(rotation_deg)
            self.print_current_pose("AFTER ROTATION")
            self.wait_real_time(1.0)
        
        # 2. Marsz do przodu (jeśli potrzebny)
        if distance > 0.01:  # Tylko jeśli odległość > 1cm
            self.get_logger().info(f'Going forward in distance {distance:.3f}m')
            wychyly_marsz = generate_walking_sequence(distance)
            self.execute_walking_sequence(wychyly_marsz)
            self.update_position_after_walking(distance)
            self.print_current_pose("AFTER MARCH")
            self.wait_real_time(1.0)
        
        self.get_logger().info('=== ENDED MOVEMENT TO POINT ===')

    def execute_complete_sequence(self):
        """executing sequence point-to-point"""
        self.get_logger().info('=== STARTING SEQUENCE POINT-TO-POINT ===')
        
        self.wait_real_time(0.5)

        coords = sys.argv[1:]
        if coords:
            if len(coords) % 2 != 0:
                self.get_logger().error("ERROR: number of args must be even (x y x y ...)")
                return
            points = []
            for i in range(0, len(coords), 2):
                try:
                    x = float(coords[i])
                    y = float(coords[i+1])
                except ValueError:
                    self.get_logger().error(f"Invalid points: {coords[i]}, {coords[i+1]}")
                    return
                points.append((x, y))
            for x, y in points:
                self.get_logger().info(f"Going to ({x}, {y})")
                self.move_to_point(x, y)
        else:
            # FIRST POINT
            self.move_to_point(0.25, 0.25)
            # SECOND POINT
            self.move_to_point(0.0, 0.25)

        self.get_logger().info('=== SEQUENCE ENDED ===')

def main(args=None):
    rclpy.init(args=args)
    
    node = CombinedHexapodController()
    
    try:
        print("=== HEXAPOD COMBINED SEQUENCE ===")
        
        # Executing full sequence
        node.execute_complete_sequence()
        
        print("All sequences ended")
        
    except KeyboardInterrupt:
        print("\nSTOP")
    
    # Cleaning
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
