#!/usr/bin/env python3

"""
This program implements a bi-gait locomotion pattern for hexapod
Bi-gate explanation:
The Hexapod moves in pairs:
    - Pair 1: legs 1 and 4 (front right & back left)
    - Pair 2: legs 2 and 5 (middle right & middle left)
    - Pair 3: legs 3 and 6 (back right & front left)
Each pair moves one after another. 

To change the height of the leg movement, modify parameter h; to change step length - r
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool  
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
    """
    Calculates servo angles (inverse kinematics) for hexapod leg.
    """
    alfa_1 = np.arctan2(P3[1], P3[0])

    P1 = np.array([l1 * np.cos(alfa_1), l1 * np.sin(alfa_1), 0])

    d = np.sqrt((P3[0] - P1[0]) ** 2 + (P3[1] - P1[1]) ** 2 + (P3[2] - P1[2]) ** 2)

    cos_fi = (l2 ** 2 + l3 ** 2 - d ** 2) / (2 * l2 * l3)
    fi = np.arccos(cos_fi)
    alfa_3 = np.deg2rad(180) - fi

    epsilon = np.arcsin(np.sin(fi) * l3 / d)
    tau = np.arctan2(P3[2] - P1[2], np.sqrt((P3[0] - P1[0]) ** 2 + (P3[1] - P1[1]) ** 2))

    alfa_2 = -(epsilon + tau)
    return [alfa_1, alfa_2, alfa_3]

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

l1 = 0.17995 - 0.12184
l2 = 0.30075 - 0.17995
l3 = 0.50975 - 0.30075

# the resting angles of the joints fitted
alfa_1 = 0
alfa_2 = np.radians(10)
alfa_3 = np.radians(80)

P0 = np.array([0, 0, 0])
P1 = P0 + np.array([l1 * np.cos(alfa_1), l1 *np.sin(alfa_1), 0])
P2 = P1 + np.array([np.cos(alfa_1)*np.cos(alfa_2)*l2,np.sin(alfa_1)*np.cos(alfa_2)*l2, np.sin(alfa_2) * l2])
P3 = P2 + np.array([np.cos(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_1)*np.cos(alfa_2 - alfa_3)*l3, np.sin(alfa_2 - alfa_3) * l3])

x_start = l1 + l2 * np.cos(alfa_2) + l3 * np.sin(np.deg2rad(90) - alfa_2 - alfa_3)  # poczatkowe wychylenie nogi pajaka w osi x
z_start = -(l2*np.sin(alfa_2) + l3 * np.cos(np.deg2rad(90) - alfa_2 - alfa_3))  # poczatkowy z

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

#steps of the legs are made relative to the point (0,0,0) that is the attachment of the leg to the torso
#such calculations mean that we don't have to do translation -> rotation -> translation to make the cycle fit each leg, and rotation alone and translation after it is enough
punkty_etap1_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, 10000, 0)
punkty_etap2_ruchu_y = np.linspace(r * (ilosc_punktow_na_krzywych - 1) / ilosc_punktow_na_krzywych, 0, ilosc_punktow_na_krzywych)
punkty_etap2_ruchu = [[0, punkty_etap2_ruchu_y[i], 0] for i in range(ilosc_punktow_na_krzywych)]
punkty_etap3_ruchu_y = np.linspace(-r / ilosc_punktow_na_krzywych, -r, ilosc_punktow_na_krzywych)
punkty_etap3_ruchu = [[0, punkty_etap3_ruchu_y[i], 0] for i in range(ilosc_punktow_na_krzywych)]
punkty_etap4_ruchu = znajdz_punkty_kwadratowe(2 * r, h, ilosc_punktow_na_krzywych, 20000, -r)
punkty_etap5_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, 10000, -r)
punkty_etap6_ruchu = punkty_etap5_ruchu.copy()
punkty_etap6_ruchu.reverse()
punkty_etap7_ruchu = punkty_etap1_ruchu.copy()
punkty_etap7_ruchu.reverse()


#zeroes are the resting points of the legs as the others settle into their starting positions
zera = []
for i in range(ilosc_punktow_na_krzywych):
    zera.append([0, 0, 0])

pierwszy_krok_nóg_1_4 = punkty_etap6_ruchu.copy()
pierwszy_krok_nóg_2_5 = zera.copy()
pierwszy_krok_nóg_3_6 = zera.copy()

#fill is a copy of the last step of setting up the legs that were set up first so that they don't change while the rest of the legs are being set up

wypelniacz = []
for i in range(ilosc_punktow_na_krzywych):
    wypelniacz.append(pierwszy_krok_nóg_1_4[ilosc_punktow_na_krzywych - 1])

drugi_krok_nóg_1_4 = wypelniacz.copy()
drugi_krok_nóg_2_5 = zera.copy()
drugi_krok_nóg_3_6 = punkty_etap1_ruchu.copy()

# Generating points on curve

cykl_nog_1_4 = pierwszy_krok_nóg_1_4.copy()
cykl_nog_2_5 = pierwszy_krok_nóg_2_5.copy()
cykl_nog_3_6 = pierwszy_krok_nóg_3_6.copy()

cykl_nog_1_4 = np.concatenate([cykl_nog_1_4, drugi_krok_nóg_1_4])
cykl_nog_2_5 = np.concatenate([cykl_nog_2_5, drugi_krok_nóg_2_5])
cykl_nog_3_6 = np.concatenate([cykl_nog_3_6, drugi_krok_nóg_3_6])

ilosc_cykli = optimal_cycles 


#Main cycle
for _ in range(ilosc_cykli):
    cykl_nog_1_4 = np.concatenate([cykl_nog_1_4, punkty_etap4_ruchu, punkty_etap2_ruchu, punkty_etap3_ruchu])
    cykl_nog_2_5 = np.concatenate([cykl_nog_2_5, punkty_etap3_ruchu, punkty_etap4_ruchu, punkty_etap2_ruchu])
    cykl_nog_3_6 = np.concatenate([cykl_nog_3_6, punkty_etap2_ruchu, punkty_etap3_ruchu, punkty_etap4_ruchu])

wypelniacz = []
for i in range(ilosc_punktow_na_krzywych):
    wypelniacz.append(punkty_etap4_ruchu[ilosc_punktow_na_krzywych - 1])

#Returning legs to home position
cykl_nog_1_4 = np.concatenate([cykl_nog_1_4, punkty_etap5_ruchu, zera])
cykl_nog_2_5 = np.concatenate([cykl_nog_2_5, zera, zera])
cykl_nog_3_6 = np.concatenate([cykl_nog_3_6, wypelniacz, punkty_etap7_ruchu])

# Update the cycle array to use the new unified cycle
#ROTATION
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
#TRANSLATION
polozenia_stop_podczas_cyklu = np.array([ 
    [[
        stopa_spoczynkowa[0] + cykle_nog[j][i][0],
        stopa_spoczynkowa[1] + cykle_nog[j][i][1],
        stopa_spoczynkowa[2] + cykle_nog[j][i][2]
    ]
    for i in range(len(cykl_nog_1_4))]
    for j in range(6)
])

#results given for 1st 2 and 3rd joint in radians respectively
wychyly_serw_podczas_ruchu = np.array([
[katy_serw(polozenia_stop_podczas_cyklu[j][i], l1, l2, l3)
    for i in range(len(cykl_nog_1_4))]
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
        
         # Publishers for controllers of all legs
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
        
        if step_index >= len(wychyly_serw_podczas_ruchu[0]):
            self.get_logger().error(f'Step index {step_index} is out of range!')
            return False
        
        # Set duration of movement
        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        # For each leg prepare and send a trajectory
        for leg_num in range(1, 7):

            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names[leg_num]
            
            point = JointTrajectoryPoint()

            joint_values = wychyly_serw_podczas_ruchu[leg_num-1][step_index]
            
            point.positions = [
                float(joint_values[0]),  # joint1
                float(joint_values[1]),  # joint2
                float(joint_values[2])   # joint3
            ]
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            
            # time of movement
            point.time_from_start = duration
            
            trajectory.points.append(point)

            self.trajectory_publishers[leg_num].publish(trajectory)
        
        return True
    
    
    def execute_sequence_with_contact_monitoring(self, start_step=0, end_step=None, step_duration=0.4):
        """
        Execute sequence with contact status monitoring for safer walking
        """
        
        if end_step is None:
            end_step = len(wychyly_serw_podczas_ruchu[0])
        
        # Initial position
        self.send_trajectory_to_all_legs_at_step(start_step, duration_sec=step_duration)
        self.get_logger().info('Waiting for execution of starting move...')
        time.sleep(step_duration)
        
        # Execute sequence with contact monitoring
        for step in range(start_step + 1, end_step):
            # Send trajectory
            self.send_trajectory_to_all_legs_at_step(step, duration_sec=step_duration)
            
            # Monitor contact status during movement
            start_time = time.time()
            while time.time() - start_time < step_duration:
                # Check stability every 0.05 seconds
                rclpy.spin_once(self, timeout_sec=0.05)
                
                # Log contact status periodically
                if int((time.time() - start_time) * 20) % 8 == 0:  # Every 0.4 seconds
                    contact_info = self.get_all_contact_status()


def main(args=None):
    rclpy.init(args=args)
    
    # creating a node
    node = LegSequencePlayer()
    
    try:
        
        print("Initialisation... Wait 0.5 seconds.")
        time.sleep(0.5)
        
        # Performing the sequence
        print("Starting the sequence with contact monitoring")
        node.execute_sequence_with_contact_monitoring()
        
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        pass
    
    # Cleaning
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
