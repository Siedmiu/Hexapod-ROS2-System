#!/usr/bin/env python3

"""
Ripple-gate explanation:
It's basically tri-gate, but non od the legs are in sync.
There only 3 legs moving at a time, and they move in a wave-like pattern.

Order of movement: 
[2,6,1,5,3,4]
The cycle looks as follows:
1) covering distance r/3 - legs 2,6,1 - leg 2 stops after this, legs 6 and 1 continue to move. Leg 5 also starts to move
2) covering distance r/3 - legs 6,1,5 - leg 6 stops after this, legs 1 and 5 continue to move. Leg 3 also starts to move...
3) distance r/3 - legs 1,5,3
4) distance r/3 - legs 5,3,4
5) distance run r/3 - legs 3,4,2
6. distance r/3 - legs 4,2,6

In total in such a cycle each leg covers distance r.
Each leg spends 50% of the cycle in motion.

Leg 1 does the movement from 0 to 1/2 the cycle time,
Leg 5 from 1/6 time to 4/6 time
Leg 3 from 2/6 time to 5/6 time
etc.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool  # Assuming contact status is Bool type
from builtin_interfaces.msg import Duration
import time
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import matplotlib.animation as animation
import argparse


matplotlib.use('TkAgg')


# =============================================================================
# INVERSE KINEMATICS AND LEG GEOMETRY FUNCTIONS
# =============================================================================

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

def znajdz_punkty_kwadratowe(r, h, ilosc_punktow_na_krzywej, bufor_y):
    """
    Generates points for square motion starting in point (0, 0, 0): up -> forward -> down
    r - movement range in Y direction
    h - lift height
    ilosc_punktow_na_krzywej - number of points on the entire trajectory
    bufor_y - Y-axis offset 
    """
    punkty = []
    
    # Divide points into 3 phases: up, forward, down

    punkty_w_gore = max(1, ilosc_punktow_na_krzywej // 4) 
    punkty_do_przodu = max(1, ilosc_punktow_na_krzywej // 2)
    punkty_w_dol = ilosc_punktow_na_krzywych - punkty_w_gore - punkty_do_przodu
    
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
    
    return np.array(punkty)


def calculate_optimal_r_and_cycles(target_distance, l3):
    """
    Calculates optimal r and number of cycles for a given distance
    target_distance = r (startup+shutdown) + cycles * 2r (main_loop)
    target_distance = r * (2*cycles)
    """

    r_max = l3 / 3  # max r
    r_min = l3 / 100  # minimal r
    
    best_r = None
    best_cycles = None
    
    for cycles in range(1, 1000):
        required_r = target_distance / (2 * cycles)
        
        if r_min <= required_r <= r_max:
            if best_r is None or required_r > best_r:
                best_r = required_r
                best_cycles = cycles
                
        if required_r < r_min:
            break
    
    return best_r, best_cycles


l1 = 0.17995 - 0.12184
l2 = 0.30075 - 0.17995
l3 = 0.50975 - 0.30075

alfa_1 = 0
alfa_2 = np.radians(10)
alfa_3 = np.radians(80)

P0 = np.array([0, 0, 0])
P1 = P0 + np.array([l1 * np.cos(alfa_1), l1 *np.sin(alfa_1), 0])
P2 = P1 + np.array([np.cos(alfa_1)*np.cos(alfa_2)*l2,np.sin(alfa_1)*np.cos(alfa_2)*l2, np.sin(alfa_2) * l2])
P3 = P2 + np.array([np.cos(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_2 - alfa_3) * l3])

x_start = l1 + l2 * np.cos(alfa_2) + l3 * np.sin(np.deg2rad(90) - alfa_2 - alfa_3)  
z_start = -(l2*np.sin(alfa_2) + l3 * np.cos(np.deg2rad(90) - alfa_2 - alfa_3))  

stopa_spoczynkowa = [x_start, 0, z_start]

nachylenia_nog_do_bokow_platformy_pajaka = np.array([
    np.deg2rad(45), 0, np.deg2rad(-45), np.deg2rad(180 + 45), np.deg2rad(180), np.deg2rad(180 - 45)
])

# the track traveled by the legs in the robot's center coordinate system
h = 0.1

target_distance = 1
optimal_r, optimal_cycles = calculate_optimal_r_and_cycles(target_distance, l3)

parser = argparse.ArgumentParser()
parser.add_argument('--back', action='store_true', help='jeśli true, idź do tyłu')
args = parser.parse_args()

r = -optimal_r if args.back else optimal_r
ilosc_punktow_na_krzywych = 10

# =============================================================================
# TRAJECTORY OF LEGS
# =============================================================================


punkty_etap1_ruchu = znajdz_punkty_kwadratowe(r, h / 2, 2*ilosc_punktow_na_krzywych, 0)
punkty_etap2_ruchu_y = np.linspace(r * (ilosc_punktow_na_krzywych - 1) / ilosc_punktow_na_krzywych, 0, 3*ilosc_punktow_na_krzywych)
punkty_etap2_ruchu = [[0, punkty_etap2_ruchu_y[i], 0] for i in range(3*ilosc_punktow_na_krzywych)]
punkty_etap3_ruchu_y = np.linspace(-r / ilosc_punktow_na_krzywych, -r, 3*ilosc_punktow_na_krzywych)
punkty_etap3_ruchu = [[0, punkty_etap3_ruchu_y[i], 0] for i in range(3*ilosc_punktow_na_krzywych)]
punkty_etap4_ruchu = znajdz_punkty_kwadratowe(2 * r, h, 12*ilosc_punktow_na_krzywych, -r)
punkty_etap5_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, -r)

ilosc_cykli = optimal_cycles # how_long_hexapod_walks
maly_krok_dlugosc = 20
#division of the cycle into 6 equal parts, half of the points being responsible for forward movement on the parabola and the other half for backward movement on the ground

cały_cykl = np.concatenate([punkty_etap2_ruchu, punkty_etap3_ruchu, punkty_etap4_ruchu])



punkt_start_przod_dol = punkty_etap1_ruchu[-1]
punkt_start_srodek_dol = punkty_etap2_ruchu[-1]
punkt_start_tyl_dol = punkty_etap3_ruchu[-1]
punkt_start_tyl_gora = punkt_start_tyl_dol + np.array([0, 0, h])
punkt_start_srodek_gora = punkt_start_srodek_dol + np.array([0,0,h])
punkt_start_przod_gora = punkt_start_przod_dol + np.array([0,0,h])

pierwszy_krok_1_nogi = punkt_start_srodek_dol
pierwszy_krok_2_nogi = punkt_start_srodek_dol
pierwszy_krok_3_nogi = punkt_start_srodek_dol
pierwszy_krok_4_nogi = punkt_start_srodek_gora
pierwszy_krok_5_nogi = punkt_start_srodek_dol
pierwszy_krok_6_nogi = punkt_start_srodek_gora

drugi_krok_1_nogi = punkt_start_srodek_dol
drugi_krok_2_nogi = punkt_start_srodek_dol
drugi_krok_3_nogi = punkt_start_srodek_dol
drugi_krok_4_nogi = punkt_start_przod_dol
drugi_krok_5_nogi = punkt_start_srodek_dol
drugi_krok_6_nogi = punkt_start_tyl_dol

trzeci_krok_1_nogi = punkt_start_przod_gora
trzeci_krok_2_nogi = punkt_start_srodek_dol
trzeci_krok_3_nogi = punkt_start_tyl_gora    
trzeci_krok_4_nogi = punkt_start_przod_dol
trzeci_krok_5_nogi = punkt_start_srodek_gora
trzeci_krok_6_nogi = punkt_start_tyl_dol

cykl_nogi_1 = np.vstack([pierwszy_krok_1_nogi, drugi_krok_1_nogi, trzeci_krok_1_nogi])
cykl_nogi_2 = np.vstack([pierwszy_krok_2_nogi, drugi_krok_2_nogi, trzeci_krok_2_nogi])
cykl_nogi_3 = np.vstack([pierwszy_krok_3_nogi, drugi_krok_3_nogi, trzeci_krok_3_nogi])
cykl_nogi_4 = np.vstack([pierwszy_krok_4_nogi, drugi_krok_4_nogi, trzeci_krok_4_nogi])
cykl_nogi_5 = np.vstack([pierwszy_krok_5_nogi, drugi_krok_5_nogi, trzeci_krok_5_nogi])
cykl_nogi_6 = np.vstack([pierwszy_krok_6_nogi, drugi_krok_6_nogi, trzeci_krok_6_nogi])

#main cycle

for _ in range(ilosc_cykli):
    # Dla każdej nogi tworzymy tablicę nowych punktów i łączymy z istniejącym cyklem
    nowe_punkty_1 = np.array([punkt_start_przod_dol, punkt_start_srodek_dol, punkt_start_tyl_dol, 
                             punkt_start_tyl_gora, punkt_start_srodek_gora, punkt_start_przod_gora])
    cykl_nogi_1 = np.vstack([cykl_nogi_1, nowe_punkty_1])
    
    nowe_punkty_2 = np.array([punkt_start_tyl_dol, punkt_start_tyl_gora, punkt_start_srodek_gora, 
                             punkt_start_przod_gora, punkt_start_przod_dol, punkt_start_srodek_dol])
    cykl_nogi_2 = np.vstack([cykl_nogi_2, nowe_punkty_2])
    
    nowe_punkty_3 = np.array([punkt_start_srodek_gora, punkt_start_przod_gora, punkt_start_przod_dol, 
                             punkt_start_srodek_dol, punkt_start_tyl_dol, punkt_start_tyl_gora])
    cykl_nogi_3 = np.vstack([cykl_nogi_3, nowe_punkty_3])
    
    nowe_punkty_4 = np.array([punkt_start_srodek_dol, punkt_start_tyl_dol, punkt_start_tyl_gora, 
                             punkt_start_srodek_gora, punkt_start_przod_gora, punkt_start_przod_dol])
    cykl_nogi_4 = np.vstack([cykl_nogi_4, nowe_punkty_4])
    
    nowe_punkty_5 = np.array([punkt_start_przod_gora, punkt_start_przod_dol, punkt_start_srodek_dol, 
                             punkt_start_tyl_dol, punkt_start_tyl_gora, punkt_start_srodek_gora])
    cykl_nogi_5 = np.vstack([cykl_nogi_5, nowe_punkty_5])
    
    nowe_punkty_6 = np.array([punkt_start_tyl_gora, punkt_start_srodek_gora, punkt_start_przod_gora, 
                             punkt_start_przod_dol, punkt_start_srodek_dol, punkt_start_tyl_dol])
    cykl_nogi_6 = np.vstack([cykl_nogi_6, nowe_punkty_6])
#return to home positions

pierwszy_od_konca_krok_1_nogi = [punkt_start_srodek_dol]
pierwszy_od_konca_krok_2_nogi = [punkt_start_srodek_dol]
pierwszy_od_konca_krok_3_nogi = [punkt_start_srodek_dol]
pierwszy_od_konca_krok_4_nogi = [punkt_start_przod_dol]
pierwszy_od_konca_krok_5_nogi = [punkt_start_srodek_dol]
pierwszy_od_konca_krok_6_nogi = [punkt_start_tyl_dol]

drugi_od_konca_krok_1_nogi = [punkt_start_srodek_dol]
drugi_od_konca_krok_2_nogi = [punkt_start_srodek_dol]
drugi_od_konca_krok_3_nogi = [punkt_start_srodek_dol]
drugi_od_konca_krok_4_nogi = [punkt_start_srodek_dol]
drugi_od_konca_krok_5_nogi = [punkt_start_srodek_dol]
drugi_od_konca_krok_6_nogi = [punkt_start_srodek_dol]

cykl_nogi_1 = np.vstack([cykl_nogi_1, pierwszy_od_konca_krok_1_nogi, drugi_od_konca_krok_1_nogi])
cykl_nogi_2 = np.vstack([cykl_nogi_2, pierwszy_od_konca_krok_2_nogi, drugi_od_konca_krok_2_nogi])
cykl_nogi_3 = np.vstack([cykl_nogi_3, pierwszy_od_konca_krok_3_nogi, drugi_od_konca_krok_3_nogi])
cykl_nogi_4 = np.vstack([cykl_nogi_4, pierwszy_od_konca_krok_4_nogi, drugi_od_konca_krok_4_nogi])
cykl_nogi_5 = np.vstack([cykl_nogi_5, pierwszy_od_konca_krok_5_nogi, drugi_od_konca_krok_5_nogi])
cykl_nogi_6 = np.vstack([cykl_nogi_6, pierwszy_od_konca_krok_6_nogi, drugi_od_konca_krok_6_nogi])

cykle_nog = np.array([
    [
        [cykl_nogi_1[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_1[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_1[i][2]]
        for i in range(len(cykl_nogi_1))
    ] if j == 0 else
    [
        [cykl_nogi_2[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_2[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_2[i][2]]
        for i in range(len(cykl_nogi_2))
    ] if j == 1 else
    [
        [cykl_nogi_3[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_3[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_3[i][2]]
        for i in range(len(cykl_nogi_3))
    ] if j == 2 else
    [
        [cykl_nogi_4[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_4[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_4[i][2]]
        for i in range(len(cykl_nogi_4))
    ] if j == 5 else
    [
        [cykl_nogi_5[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_5[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_5[i][2]]
        for i in range(len(cykl_nogi_5))
    ] if j == 4 else
    [
        [cykl_nogi_6[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_6[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
         cykl_nogi_6[i][2]]
        for i in range(len(cykl_nogi_6))
    ] 
    for j in range(6)

])

polozenia_stop_podczas_cyklu = np.array([
    [[
        stopa_spoczynkowa[0] + cykle_nog[j][i][0],
        stopa_spoczynkowa[1] + cykle_nog[j][i][1],
        stopa_spoczynkowa[2] + cykle_nog[j][i][2]
    ]
    for i in range(len(cykl_nogi_1))]
    for j in range(6)
])

wychyly_serw_podczas_ruchu = np.array([
[katy_serw(polozenia_stop_podczas_cyklu[j][i], l1, l2, l3)
    for i in range(len(cykl_nogi_1))]
    for j in range(6)
])

class LegSequencePlayer(Node):
    def __init__(self):
        super().__init__('leg_sequence_player')
        self.get_logger().info('Inicjalizacja węzła do sekwencji ruchów')
        
        # Contact status storage for each leg
        self.contact_status = {
            1: False, 2: False, 3: False, 
            4: False, 5: False, 6: False
        }
    
        self.trajectory_publishers = {
            1: self.create_publisher(JointTrajectory, '/leg1_controller/joint_trajectory', 10),
            2: self.create_publisher(JointTrajectory, '/leg2_controller/joint_trajectory', 10),
            3: self.create_publisher(JointTrajectory, '/leg3_controller/joint_trajectory', 10),
            4: self.create_publisher(JointTrajectory, '/leg4_controller/joint_trajectory', 10),
            5: self.create_publisher(JointTrajectory, '/leg5_controller/joint_trajectory', 10),
            6: self.create_publisher(JointTrajectory, '/leg6_controller/joint_trajectory', 10)
        }
        
        # Contact status subscribers
        self.contact_subscribers = {
            1: self.create_subscription(Bool, '/hexapod/leg1/contact_status', 
                                      lambda msg, leg=1: self.contact_callback(msg, leg), 10),
            2: self.create_subscription(Bool, '/hexapod/leg2/contact_status', 
                                      lambda msg, leg=2: self.contact_callback(msg, leg), 10),
            3: self.create_subscription(Bool, '/hexapod/leg3/contact_status', 
                                      lambda msg, leg=3: self.contact_callback(msg, leg), 10),
            4: self.create_subscription(Bool, '/hexapod/leg4/contact_status', 
                                      lambda msg, leg=4: self.contact_callback(msg, leg), 10),
            5: self.create_subscription(Bool, '/hexapod/leg5/contact_status', 
                                      lambda msg, leg=5: self.contact_callback(msg, leg), 10),
            6: self.create_subscription(Bool, '/hexapod/leg6/contact_status', 
                                      lambda msg, leg=6: self.contact_callback(msg, leg), 10)
        }
        
        self.joint_names = {
            1: ['joint1_1', 'joint2_1', 'joint3_1'],
            2: ['joint1_2', 'joint2_2', 'joint3_2'],
            3: ['joint1_3', 'joint2_3', 'joint3_3'],
            4: ['joint1_4', 'joint2_4', 'joint3_4'],
            5: ['joint1_5', 'joint2_5', 'joint3_5'],
            6: ['joint1_6', 'joint2_6', 'joint3_6']
        }
        
    def contact_callback(self, msg, leg_number):
        """
        Callback function for contact status messages
        """
        self.contact_status[leg_number] = msg.data
        self.get_logger().debug(f'Noga {leg_number} kontakt: {msg.data}')
    
    def get_contact_status(self, leg_number):
        """
        Get current contact status for specific leg
        """
        return self.contact_status.get(leg_number, False)
    
    def get_all_contact_status(self):
        """
        Get contact status for all legs
        """
        return self.contact_status.copy()
    
    def wait_for_contact(self, leg_number, expected_status=True, timeout=5.0):
        """
        Wait until specific leg reaches expected contact status
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.contact_status[leg_number] == expected_status:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def send_trajectory_to_all_legs_at_step(self, step_index, duration_sec=2.0):
        """
        Sends the trajectory to the controllers of all legs simultaneously
        using the values from the outcrop_serv_motion array for a given step
        """

        self.get_logger().info(f'Wysyłam trajektorię dla kroku {step_index}')
        
        if step_index >= len(wychyly_serw_podczas_ruchu[0]):
            self.get_logger().error(f'Indeks kroku {step_index} jest poza zakresem!')
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
        
        self.get_logger().info('Send trajectory to all legs')
        return True
    
    def execute_sequence_with_contact_monitoring(self, start_step=0, end_step=None, step_duration=2.0):
        """
        Execute sequence with contact status monitoring for safer walking
        """
        self.get_logger().info('Starting the sequence with contact monitoring')
        
        if end_step is None:
            end_step = len(wychyly_serw_podczas_ruchu[0])
        
        self.send_trajectory_to_all_legs_at_step(start_step, duration_sec=3.0)
        self.get_logger().info('Waiting for the initial move to be made....')
        time.sleep(0.5)
        
        for step in range(start_step + 1, end_step):
            self.send_trajectory_to_all_legs_at_step(step, duration_sec=step_duration)
            
            start_time = time.time()
            while time.time() - start_time < step_duration:
                rclpy.spin_once(self, timeout_sec=0.05)
                
                if int((time.time() - start_time) * 20) % 4 == 0:  
                    contact_info = self.get_all_contact_status()
                    self.get_logger().info(f'Krok {step}, Kontakt: {contact_info}')
            
            self.get_logger().info(f'Step {step} performed with monitoring, waiting {step_duration}s...')
        
        self.get_logger().info('Sequence with monitoring ended')


def main(args=None):
    rclpy.init(args=args)
    
    node = LegSequencePlayer()
    
    try:
        
        print("Initialisation...")
        time.sleep(0.5)
        
        node.execute_sequence_with_contact_monitoring()
        
        time.sleep(0.5)
        
    except KeyboardInterrupt:
        pass
    
    # Cleaning
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()