#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import matplotlib.animation as animation
import argparse

matplotlib.use('TkAgg')

# --- parsing the argument for the angle of rotation ---
parser = argparse.ArgumentParser(description='Rotation for hexapod')
parser.add_argument('--angle', type=float, default=90.0,
                    help='full angle in degrees')
args = parser.parse_args()

def turn_hexapod(R, alfa, x_start, z):
    x_new = (x_start + R) * np.cos(alfa) - R
    y_new = (x_start + R) * np.sin(alfa)

    return np.array([x_new, y_new, z])

def trajektoria_prostokatna(start, cel, h, liczba_punktow):
    liczba_punktow += 3
    start_gora = start + np.array([0, 0, h])
    cel_gora = cel + np.array([0, 0, h])

    etap1 = np.linspace(start, start_gora, liczba_punktow // 3)
    etap2 = np.linspace(start_gora, cel_gora, liczba_punktow // 3)
    etap3 = np.linspace(cel_gora, cel, liczba_punktow - len(etap1) - len(etap2))

    punkty = np.concatenate([etap1[1:], etap2[1:], etap3[1:]], axis=0)
    return punkty

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

l1 = 0.17995 - 0.12184
l2 = 0.30075 - 0.17995
l3 = 0.50975 - 0.30075

#if they are changed they have to be changed also in yaml
alfa_1 = 0
alfa_2 = np.deg2rad(10)
alfa_3 = np.deg2rad(80)

stala_naprawcza = 1

kat_calkowity = np.radians(args.angle)

odleglosc_przegubow_od_srodka_hexapoda = 0.1218
kat_obrotu_cyklu = np.radians(20)
kat_obrotu = kat_obrotu_cyklu / 2 * stala_naprawcza

#direction of rotation
kierunek = np.sign(kat_calkowity)
kat_obrotu *= kierunek
#condition for spinning is alpha1 = 0
x_start = l1 + l2 * np.cos(alfa_2) + l3 * np.sin(np.deg2rad(90) - alfa_2 - alfa_3)  # starting x
z_start = -(l2*np.sin(alfa_2) + l3 * np.cos(np.deg2rad(90) - alfa_2 - alfa_3))  # starting z
h = 0.1  
ilosc_punktow_na_etap = 30

punkt_start_dla_kazdej_nogi = [x_start, 0, z_start]
punkt_P1 = turn_hexapod(odleglosc_przegubow_od_srodka_hexapoda, kat_obrotu, x_start, z_start)
punkt_P2 = turn_hexapod(odleglosc_przegubow_od_srodka_hexapoda, -kat_obrotu, x_start, z_start)
punkt_szczytowy_etapu_1 = (punkt_start_dla_kazdej_nogi + punkt_P1) / 2 + np.array([0, 0, h])
punkt_szczytowy_etapu_5 = (punkt_start_dla_kazdej_nogi + punkt_P2) / 2 + np.array([0, 0, h])

#etap 1 from initial position to P1 in the air
#etap 2 from P1 to initial position on the ground
#etap 3 from initial position to P2 on the ground
#etap 4 possible transition from P2 to P1 in the air
#etap 5 from P2 to initial position in the air

etap_1 = np.array(trajektoria_prostokatna(punkt_start_dla_kazdej_nogi, punkt_P1, h, ilosc_punktow_na_etap))
etap_2 = np.linspace(punkt_P1, punkt_start_dla_kazdej_nogi, ilosc_punktow_na_etap)[1:]
etap_3 = np.linspace(punkt_start_dla_kazdej_nogi, punkt_P2, ilosc_punktow_na_etap)[1:]
#etap_4 can be made to make full movement not just half
etap_5 = np.array(trajektoria_prostokatna(punkt_P2, punkt_start_dla_kazdej_nogi, h, ilosc_punktow_na_etap))

ilosc_cykli = int(np.abs(kat_calkowity) // kat_obrotu_cyklu)

pozostaly_kat = (np.abs(kat_calkowity) % kat_obrotu_cyklu) / 2 * stala_naprawcza * kierunek

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
    #etap_4 can be made to make full movement not just half
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

czy_wyswietlic = False
if czy_wyswietlic:

    plt.figure(figsize=(14, 6))

    plt.subplot(1, 2, 1)
    plt.plot(wychyly_serw_1_3_5[:, 0], label='alfa_1')
    plt.plot(wychyly_serw_1_3_5[:, 1], label='alfa_2')
    plt.plot(wychyly_serw_1_3_5[:, 2], label='alfa_3')
    plt.title('Legs 1, 3, 5')
    plt.xlabel('Step')
    plt.ylabel('Angle (radians)')
    plt.legend()
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.plot(wychyly_serw_2_4_6[:, 0], label='alfa_1')
    plt.plot(wychyly_serw_2_4_6[:, 1], label='alfa_2')
    plt.plot(wychyly_serw_2_4_6[:, 2], label='alfa_3')
    plt.title('Legs 2, 4, 6')
    plt.xlabel('Step')
    plt.ylabel('Angle (radians)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

class LegSequencePlayer(Node):
    def __init__(self):
        super().__init__('leg_sequence_player')
        self.get_logger().info('Initialisation...')
        
        # Przechowaj tablicę z wychyłami serw
        
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
        

    def send_trajectory_to_all_legs_at_step(self, step_index, duration_sec=2.0):
        self.get_logger().info(f'Sending trajectory for step {step_index}')

        if step_index >= len(wychyly_serw_1_3_5) or step_index >= len(wychyly_serw_2_4_6):
            self.get_logger().error(f'Index of step {step_index} out of range!')
            return False

        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        for leg_num in range(1, 7):
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names[leg_num]

            point = JointTrajectoryPoint()

            if leg_num in [1, 3, 5]:
                joint_values = wychyly_serw_1_3_5[step_index]
            elif leg_num in [2, 4, 6]:
                joint_values = wychyly_serw_2_4_6[step_index]
            else:
                self.get_logger().error(f'Invalid leg number: {leg_num}')
                continue

            point.positions = list(map(float, joint_values))
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            point.time_from_start = duration

            trajectory.points.append(point)

            self.trajectory_publishers[leg_num].publish(trajectory)

        self.get_logger().info('Sending trajectory')
        return True

    
    def execute_sequence(self, start_step=0, end_step=None, step_duration=0.2):
        """
        Perform a sequence of movements for all legs simultaneously
        using outriggers from separate tables: outriggers_serw_1_3_5 and outriggers_serw_2_4_6
        """
        self.get_logger().info('Movement started')
        
        # Zakładamy, że tablice mają tę samą długość
        max_steps = min(len(wychyly_serw_1_3_5), len(wychyly_serw_2_4_6))

        if end_step is None or end_step > max_steps:
            end_step = max_steps

        # Przejście do pozycji początkowej (pierwszy punkt)
        self.send_trajectory_to_all_legs_at_step(start_step, duration_sec=0.2)

        self.get_logger().info('Waiting for first movement...')
        time.sleep(0.2)

        for step in range(start_step + 1, end_step):
            self.send_trajectory_to_all_legs_at_step(step, duration_sec=step_duration)
            self.get_logger().info(f'Ended step {step}, waiting {step_duration}s...')
            time.sleep(step_duration)

        self.get_logger().info('Sequence ended')



def main(args=None):
    rclpy.init(args=args)
    
    # Utworzenie węzła
    node = LegSequencePlayer()
    
    try:
        
        print("Initialisation...")
        time.sleep(0.5)
        
        node.execute_sequence()
        
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        pass
    
    # Cleaning
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
