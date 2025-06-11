#!/usr/bin/env python3

"""

This program implements tri-gate using contact sensors to avoid

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
from mpl_toolkits.mplot3d import Axes3D
import threading


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
    cos_fi = np.clip(cos_fi, -1.0, 1.0)
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
    r_max = l3 / 3  # max r 
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

# Location of the resting point from the leg attachment determined based on the angle of the joints at rest
# IMPORTANT !!! this is the location of the foot in the foot attachment point alignment and not the XYZ alignment
# in which X1 is the perpendicular straight line to the side of the platform to which the leg is attached and grows in the direction from the hexapod
# Y1 is the axis that coincides with the side of the platform to which the leg is attached and grows toward the front of the hexapod
# Z1 coincides with the Z axis of the XYZ system


alfa_1 = 0
alfa_2 = np.radians(10)
alfa_3 = np.radians(80)

x_start = l1 + l2 * np.cos(alfa_2) + l3 * np.sin(np.deg2rad(90) - alfa_2 - alfa_3)  # starting x
z_start = -(l2*np.sin(alfa_2) + l3 * np.cos(np.deg2rad(90) - alfa_2 - alfa_3))  # starting z

stopa_spoczynkowa = np.array([x_start, 0, z_start])

nachylenia_nog_do_bokow_platformy_pajaka = np.array([
    np.deg2rad(45), 0, np.deg2rad(-45), np.deg2rad(180 + 45), np.deg2rad(180), np.deg2rad(180 - 45)
])

target_distance = 1
optimal_r, optimal_cycles = calculate_optimal_r_and_cycles(target_distance, l3)
polozenie_stop = np.array([stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa])

def step_up(p1, delta_h):
    """
    Generates upward movement by 1 step
    """
    return p1 + np.array([0, 0, delta_h])


def step_forward(p1, delta_r):
    """
    Generates forward movement by 1 step
    """
    return p1 + np.array([0, delta_r, 0])

def step_backward(p1, delta_r):
    """
    Generates forward movement by 1 step
    """
    return p1 + np.array([0, -delta_r, 0])


def step_down(p1, delta_h):
    """
    Generates downward movement by 1 step
    """
    return p1 + np.array([0, 0, -delta_h])

def counting_point_for_each_leg(p1, nachylenie, p_zero):
    """
    Rotates the point p1 around the Z-axis relative to the point p_zero
    by the angle 'slope' (in radians).
    """
    # Translation to the system relative to the reference point
    pt = p1 - p_zero

    # Rotation around Z axis (because y is fixed in forward motion)
    x = pt[1] * np.sin(nachylenie)
    y = pt[1] * np.cos(nachylenie)
    z = pt[2]

    # Translation back
    rotated = np.array([x, y, z]) + p_zero
    return rotated

""" 

1 legs go backward/upward the moment they are all on the ground
2 when a leg has fired the sensor it waits for all of them in their places or sensors
3 when a leg has reached the target state and one has not yet fired the sensor, it waits until it fires

START
legs 1 3 5: up --> front --> down (waits until all in position 2 4 6 and 1 3 5 pressed)
legs 2 4 6: straight back (waits until all in position 2 4 6 and 1 3 5 pressed)

FIGHT
legs 1 3 5: straight back (waits until all in seats 1 3 5 and 2 4 6 pressed) --> up --> front --> down (waits until all in seats 2 4 6 and 1 3 5 pressed)
legs 2 4 6: up--> front --> down (waits until all in seats 1 3 5 and 2 4 6 pressed) --> straight back (waits until all in seats 2 4 6 and 1 3 5 pressed)

END
legs 1 3 5: on the ground to the center
legs 2 4 6: up --> front to the center --> down
"""

class LegSequencePlayer(Node):
    def __init__(self):
        super().__init__('leg_sequence_player')
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

        # Contact status storage for each leg
        self.contact_status = {
            1: False, 2: False, 3: False, 
            4: False, 5: False, 6: False
        }
        
        self.trajectory_publishers = {
            1: self.create_publisher(JointTrajectory, '/leg1_controller/joint_trajectory', 50),
            2: self.create_publisher(JointTrajectory, '/leg2_controller/joint_trajectory', 50),
            3: self.create_publisher(JointTrajectory, '/leg3_controller/joint_trajectory', 50),
            4: self.create_publisher(JointTrajectory, '/leg4_controller/joint_trajectory', 50),
            5: self.create_publisher(JointTrajectory, '/leg5_controller/joint_trajectory', 50),
            6: self.create_publisher(JointTrajectory, '/leg6_controller/joint_trajectory', 50)
        }
        
        # Contact status subscribers
        self.contact_subscribers = {
            1: self.create_subscription(Bool, '/hexapod/leg1/contact_status', 
                                      lambda msg, leg=1: self.contact_callback(msg, leg), 1),
            2: self.create_subscription(Bool, '/hexapod/leg2/contact_status', 
                                      lambda msg, leg=2: self.contact_callback(msg, leg), 1),
            3: self.create_subscription(Bool, '/hexapod/leg3/contact_status', 
                                      lambda msg, leg=3: self.contact_callback(msg, leg), 1),
            4: self.create_subscription(Bool, '/hexapod/leg4/contact_status', 
                                      lambda msg, leg=4: self.contact_callback(msg, leg), 1),
            5: self.create_subscription(Bool, '/hexapod/leg5/contact_status', 
                                      lambda msg, leg=5: self.contact_callback(msg, leg), 1),
            6: self.create_subscription(Bool, '/hexapod/leg6/contact_status', 
                                      lambda msg, leg=6: self.contact_callback(msg, leg), 1)
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
    
    def wait_for_contact(self, leg_number, expected_status=True, timeout=0.2):
        """
        Wait until specific leg reaches expected contact status
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.contact_status[leg_number] == expected_status:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


    def execute_single_move(self, leg_positions, duration_sec=0.2):
        """
        Performs one movement and returns the contact status of all legs
        
        Args:
        leg_positions: dict {leg_num: [x, y, z], ...} or a list of 6 positions
        duration_sec: duration of movement
        
        Returns:
        dict: {leg_num: contact_status, ...}
        """
        if isinstance(leg_positions, list) and len(leg_positions) == 6:
            leg_positions = {i+1: leg_positions[i] for i in range(6)}
        
        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        for leg_num in range(1, 7):
            contact_info = self.get_contact_status(leg_num)
            if leg_num in leg_positions:
                try:
                    joint_angles = katy_serw(leg_positions[leg_num], l1, l2, l3)
                    trajectory = JointTrajectory()
                    trajectory.joint_names = self.joint_names[leg_num]
                    
                    point = JointTrajectoryPoint()
                    point.positions = [float(angle) for angle in joint_angles]
                    point.velocities = [0.0] * 3
                    point.accelerations = [0.0] * 3
                    point.time_from_start = duration 
                    
                    trajectory.points.append(point)
                    self.trajectory_publishers[leg_num].publish(trajectory)
                    
                except Exception as e:
                    self.get_logger().error(f'Error for leg {leg_num}: {e}')

        time.sleep(duration_sec + 0.1)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.contact_status.copy()


def main(args=None):
    rclpy.init(args=args)
    
    node = LegSequencePlayer()

    try:
        print("Initialisation...")
        time.sleep(0.5)
        
        print("Starting the sequence with contact monitoring")

        wysokosc_podnoszenia = 0.1
        dlugosc_kroku = 0.08

        h = wysokosc_podnoszenia
        r = optimal_r

        ilosc_punktow = 5
        ilosc_punktow_dol = 30

        ilosc_punktow_tyl = 2*ilosc_punktow+ilosc_punktow_dol

        kroczek_gora = h/ilosc_punktow
        kroczek_tyl = 4 * r/ilosc_punktow_tyl
        kroczek_dol = h/ilosc_punktow_dol

        leg_positions = [stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa]
        point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1], stopa_spoczynkowa) for leg, pos in enumerate(leg_positions, start=1)]   

        pozycje_nog_koniec = [[]for i in range (6)]

        for i in range(len(pozycje_nog_koniec)):
            pozycje_nog_koniec[i] = step_backward(leg_positions[i], r)


        # =============================================================================
        # LEGS TRAJECTORY
        # =============================================================================


        while(h > 0):
            for leg in range(1, 7):
                if leg in [1,3,5]:
                    leg_positions[leg-1] = step_up(leg_positions[leg-1], kroczek_gora)
                else:
                    leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
            point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1], stopa_spoczynkowa) for leg, pos in enumerate(leg_positions, start=1)]
            node.execute_single_move(point_positions)
            h -= kroczek_gora        

        
        h = wysokosc_podnoszenia
        gorny_kroczek_przod = r/ilosc_punktow

        while(r > 0):
            for leg in range(1, 7):
                if leg in [1, 3, 5]:
                    leg_positions[leg-1] = step_forward(leg_positions[leg-1], gorny_kroczek_przod)
                else:
                    leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
            point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1], stopa_spoczynkowa) for leg, pos in enumerate(leg_positions, start=1)]
            node.execute_single_move(point_positions)
            r -= gorny_kroczek_przod

        r = dlugosc_kroku
        czy_na_miejscu = [False for i in range (6)]

        while not all(czy_na_miejscu):
            for leg in range(1, 7):
                if leg  in [1, 3, 5]: 
                    if not node.get_contact_status(leg):
                        leg_positions[leg-1] = step_down(leg_positions[leg-1], kroczek_dol)
                else: 
                    if not czy_na_miejscu[leg-1]:
                        leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
                        if leg_positions[leg-1][1] < pozycje_nog_koniec[leg-1][1]:
                            czy_na_miejscu[leg-1] = True
            point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1], stopa_spoczynkowa) for leg, pos in enumerate(leg_positions, start=1)]
            node.execute_single_move(point_positions)
            for noga in [1, 3, 5]:
                if node.get_contact_status(noga):
                    czy_na_miejscu[noga-1] = True


        ilosc_cykli = 3

        for i in range(ilosc_cykli):
            for main_legs in [[2,4,6], [1,3,5]]:
                while(h > 0):
                    for leg in range(1, 7):
                        if leg in main_legs:
                            leg_positions[leg-1] = step_up(leg_positions[leg-1], kroczek_gora)
                        else:
                            leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
                    point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1], stopa_spoczynkowa) for leg, pos in enumerate(leg_positions, start=1)]
                    node.execute_single_move(point_positions)
                    h -= kroczek_gora
                        
                h = wysokosc_podnoszenia

                R = 2 * r
                while(R > 0):
                    for leg in range(1, 7):
                        if leg in main_legs:
                            leg_positions[leg-1] = step_forward(leg_positions[leg-1], gorny_kroczek_przod)
                        else:
                            leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
                    point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1], stopa_spoczynkowa) for leg, pos in enumerate(leg_positions, start=1)]
                    node.execute_single_move(point_positions)
                    R -= gorny_kroczek_przod

                czy_na_miejscu = [False for i in range (6)]

                while not all(czy_na_miejscu):        
                    for leg in range(1, 7):
                        if leg in main_legs:
                            if not node.get_contact_status(leg):
                                leg_positions[leg-1] = step_down(leg_positions[leg-1], kroczek_dol)
                        else: 
                            if not czy_na_miejscu[leg-1]:
                                leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
                                if leg_positions[leg-1][1] < pozycje_nog_koniec[leg-1][1]:
                                    czy_na_miejscu[leg-1] = True
                    point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1], stopa_spoczynkowa) for leg, pos in enumerate(leg_positions, start=1)]
                    node.execute_single_move(point_positions) 
                    for noga in main_legs:
                        if node.get_contact_status(noga):
                            czy_na_miejscu[noga-1] = True

        while(h > 0):
            for leg in range(1, 7):
                if leg in [2,4,6]:
                    leg_positions[leg-1] = step_up(leg_positions[leg-1], kroczek_gora)
                else:
                    leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
            point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1], stopa_spoczynkowa) for leg, pos in enumerate(leg_positions, start=1)]
            node.execute_single_move(point_positions)
            h -= kroczek_gora        

        h = wysokosc_podnoszenia
        gorny_kroczek_przod = r/ilosc_punktow

        while(r > 0):
            for leg in range(1, 7):
                if leg in [2, 4, 6]:
                    leg_positions[leg-1] = step_forward(leg_positions[leg-1], gorny_kroczek_przod)
                else:
                    leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
            point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1], stopa_spoczynkowa) for leg, pos in enumerate(leg_positions, start=1)]
            node.execute_single_move(point_positions)
            r -= gorny_kroczek_przod

        r = dlugosc_kroku

        czy_na_miejscu = [False for i in range (6)]

        while not all(czy_na_miejscu):
            for leg in range(1, 7):
                if leg  in [2, 4, 6]:
                    if not node.get_contact_status(leg):
                        leg_positions[leg-1] = step_down(leg_positions[leg-1], kroczek_dol)
                else:
                    if not czy_na_miejscu[leg-1]:
                        leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
                        if leg_positions[leg-1][1] < pozycje_nog_koniec[leg-1][1]:
                            czy_na_miejscu[leg-1] = True
            point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1], stopa_spoczynkowa) for leg, pos in enumerate(leg_positions, start=1)]
            node.execute_single_move(point_positions)
            for noga in [1, 3, 5]:
                if node.get_contact_status(noga):
                    czy_na_miejscu[noga-1] = True


        time.sleep(0.5)
        
    except KeyboardInterrupt:
        pass
    
    # Cleaning
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()