#!/usr/bin/env python3
# Prosty taniec hexapoda - machanie środkowymi nogami (2 i 5)

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rosgraph_msgs.msg import Clock
import time
import numpy as np

def generate_simple_middle_leg_wave_dance(duration=6.0, wave_steps=20):
    """
    Generuje prostą sekwencję tańca gdzie środkowe nogi (2 i 5) machają,
    a pozostałe nogi (1, 3, 4, 6) pozostają w pozycji spoczynkowej
    """
    
    # Pozycja spoczynkowa dla wszystkich nóg
    spocz_1 = 0.0                    # 0°
    spocz_2 = np.deg2rad(10)         # 10°
    spocz_3 = np.deg2rad(80)         # 80°
    
    # Pozycje machania dla środkowych nóg
    # Podniesienie nogi w górę
    wave_up_1 = 0.0                  # coxa pozostaje prosto
    wave_up_2 = np.deg2rad(-30)      # femur w górę
    wave_up_3 = np.deg2rad(45)       # tibia zgięta
    
    # Machanie w lewo/prawo
    wave_left_1 = np.deg2rad(-25)    # coxa w lewo
    wave_left_2 = np.deg2rad(-30)    # femur w górze
    wave_left_3 = np.deg2rad(45)     # tibia zgięta
    
    wave_right_1 = np.deg2rad(25)    # coxa w prawo
    wave_right_2 = np.deg2rad(-30)   # femur w górze
    wave_right_3 = np.deg2rad(45)    # tibia zgięta
    
    # Tworzenie sekwencji
    sequence = []
    
    # Faza 1: Pozycja startowa (wszystkie nogi w spoczynku)
    start_pose = [
        [spocz_1, spocz_2, spocz_3],  # Noga 1
        [spocz_1, spocz_2, spocz_3],  # Noga 2 
        [spocz_1, spocz_2, spocz_3],  # Noga 3
        [spocz_1, spocz_2, spocz_3],  # Noga 4
        [spocz_1, spocz_2, spocz_3],  # Noga 5
        [spocz_1, spocz_2, spocz_3]   # Noga 6
    ]
    
    # Dodaj pozycję startową
    for _ in range(10):  # Trzymaj przez chwilę
        sequence.append([pose.copy() for pose in start_pose])
    
    # Faza 2: Podniesienie środkowych nóg
    lift_pose = [
        [spocz_1, spocz_2, spocz_3],           # Noga 1 - spoczynek
        [wave_up_1, wave_up_2, wave_up_3],    # Noga 2 - w górę
        [spocz_1, spocz_2, spocz_3],           # Noga 3 - spoczynek
        [spocz_1, spocz_2, spocz_3],           # Noga 4 - spoczynek
        [wave_up_1, wave_up_2, wave_up_3],    # Noga 5 - w górę
        [spocz_1, spocz_2, spocz_3]            # Noga 6 - spoczynek
    ]
    
    # Dodaj podniesienie
    for _ in range(15):
        sequence.append([pose.copy() for pose in lift_pose])
    
    # Faza 3: Machanie środkowymi nogami
    for cycle in range(3):  # 3 cykle machania
        # Machanie w lewo
        wave_pose_left = [
            [spocz_1, spocz_2, spocz_3],              # Noga 1 - spoczynek
            [wave_left_1, wave_left_2, wave_left_3], # Noga 2 - w lewo
            [spocz_1, spocz_2, spocz_3],              # Noga 3 - spoczynek
            [spocz_1, spocz_2, spocz_3],              # Noga 4 - spoczynek
            [wave_right_1, wave_right_2, wave_right_3], # Noga 5 - w prawo (przeciwnie)
            [spocz_1, spocz_2, spocz_3]               # Noga 6 - spoczynek
        ]
        
        for _ in range(wave_steps):
            sequence.append([pose.copy() for pose in wave_pose_left])
        
        # Powrót do środka
        for _ in range(5):
            sequence.append([pose.copy() for pose in lift_pose])
        
        # Machanie w prawo
        wave_pose_right = [
            [spocz_1, spocz_2, spocz_3],               # Noga 1 - spoczynek
            [wave_right_1, wave_right_2, wave_right_3], # Noga 2 - w prawo
            [spocz_1, spocz_2, spocz_3],               # Noga 3 - spoczynek
            [spocz_1, spocz_2, spocz_3],               # Noga 4 - spoczynek
            [wave_left_1, wave_left_2, wave_left_3],  # Noga 5 - w lewo (przeciwnie)
            [spocz_1, spocz_2, spocz_3]                # Noga 6 - spoczynek
        ]
        
        for _ in range(wave_steps):
            sequence.append([pose.copy() for pose in wave_pose_right])
        
        # Powrót do środka
        for _ in range(5):
            sequence.append([pose.copy() for pose in lift_pose])
    
    # Faza 4: Opuszczenie środkowych nóg
    for _ in range(15):
        sequence.append([pose.copy() for pose in start_pose])
    
    # Konwersja do formatu [noga][krok][joint]
    wychyly_serw_dance = []
    for leg in range(6):
        leg_sequence = []
        for step in sequence:
            leg_sequence.append(step[leg])
        wychyly_serw_dance.append(leg_sequence)
    
    return np.array(wychyly_serw_dance)

class SimpleHexapodDance(Node):
    def __init__(self):
        super().__init__('simple_hexapod_dance')
        self.get_logger().info('  Prosty taniec hexapoda - machanie środkowymi nogami')
        
        # Subskrypcja czasu symulacji
        self.clock_subscriber = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        
        self.sim_time = None
        
        # Wydawcy dla kontrolerów nóg
        self.trajectory_publishers = {
            1: self.create_publisher(JointTrajectory, '/leg1_controller/joint_trajectory', 10),
            2: self.create_publisher(JointTrajectory, '/leg2_controller/joint_trajectory', 10),
            3: self.create_publisher(JointTrajectory, '/leg3_controller/joint_trajectory', 10),
            4: self.create_publisher(JointTrajectory, '/leg4_controller/joint_trajectory', 10),
            5: self.create_publisher(JointTrajectory, '/leg5_controller/joint_trajectory', 10),
            6: self.create_publisher(JointTrajectory, '/leg6_controller/joint_trajectory', 10)
        }
        
        # Nazwy jointów
        self.joint_names = {
            1: ['joint1_1', 'joint2_1', 'joint3_1'],
            2: ['joint1_2', 'joint2_2', 'joint3_2'],
            3: ['joint1_3', 'joint2_3', 'joint3_3'],
            4: ['joint1_4', 'joint2_4', 'joint3_4'],
            5: ['joint1_5', 'joint2_5', 'joint3_5'],
            6: ['joint1_6', 'joint2_6', 'joint3_6']
        }

    def clock_callback(self, msg):
        """Callback czasu symulacji"""
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def wait_sim_time(self, duration_sec):
        """Czeka określony czas w symulacji"""
        if self.sim_time is None:
            time.sleep(duration_sec)
            return
            
        start_time = self.sim_time
        target_time = start_time + duration_sec
        
        while self.sim_time < target_time:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.sim_time is None:
                break

    def print_leg_positions(self, step_index, wychyly_serw_array):
        """Wyświetla pozycje nóg dla debugowania"""
        print(f"\n--- Krok {step_index} ---")
        for leg_num in range(6):
            angles = wychyly_serw_array[leg_num][step_index]
            status = " SPOCZYNEK" if leg_num not in [1, 4] else " MACHANIE"
            if leg_num in [1, 4]:  # nogi 2 i 5 (indeksy 1 i 4)
                if abs(angles[0]) > 0.1:  # jeśli coxa jest wychylona
                    direction = "  LEWO" if angles[0] < 0 else "  PRAWO"
                    status = f" {direction}"
                elif abs(angles[1] - np.deg2rad(10)) > 0.2:  # jeśli femur jest podniesiony
                    status = " W GÓRZE"
            
            print(f"Noga {leg_num+1}: {status} - "
                  f"J1:{np.rad2deg(angles[0]):5.1f}° "
                  f"J2:{np.rad2deg(angles[1]):5.1f}° "
                  f"J3:{np.rad2deg(angles[2]):5.1f}°")

    def send_trajectory_step(self, wychyly_serw_array, step_index, duration_sec=0.1):
        """Wysyła trajektorię dla jednego kroku"""
        if step_index >= len(wychyly_serw_array[0]):
            return False
        
        # Wyświetl pozycje co kilka kroków
        if step_index % 15 == 0:
            self.print_leg_positions(step_index, wychyly_serw_array)
        
        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        for leg_num in range(1, 7):
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names[leg_num]
            
            point = JointTrajectoryPoint()
            joint_values = wychyly_serw_array[leg_num-1][step_index]
            
            point.positions = [float(joint_values[0]), float(joint_values[1]), float(joint_values[2])]
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            point.time_from_start = duration
            
            trajectory.points.append(point)
            self.trajectory_publishers[leg_num].publish(trajectory)
        
        return True

    def execute_simple_dance(self):
        """Wykonuje prosty taniec - machanie środkowymi nogami"""
        self.get_logger().info('  Generuję sekwencję machania środkowymi nogami...')
        
        # Wygeneruj sekwencję tańca
        dance_sequence = generate_simple_middle_leg_wave_dance(duration=8.0, wave_steps=25)
        total_steps = len(dance_sequence[0])
        
        self.get_logger().info(f'Sekwencja ma {total_steps} kroków')
        self.get_logger().info('Plan tańca:')
        self.get_logger().info('   1. Start - wszystkie nogi w spoczynku')
        self.get_logger().info('   2. Podniesienie środkowych nóg (2 i 5)')
        self.get_logger().info('   3. Machanie - 3 cykle lewo-prawo')
        self.get_logger().info('   4. Opuszczenie nóg do spoczynku')
        
        # Czekaj na inicjalizację czasu
        while self.sim_time is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('🎬 Rozpoczynam taniec!')
        
        # Wykonaj sekwencję
        step_duration = 0.08  # 80ms na krok
        for step in range(total_steps):
            self.send_trajectory_step(dance_sequence, step, step_duration)
            
            if step % 20 == 0:
                progress = (step / total_steps) * 100
                self.get_logger().info(f' Postęp: {progress:.1f}% (krok {step}/{total_steps})')
            
            self.wait_sim_time(step_duration)
        
        self.get_logger().info('Taniec zakończony! Środkowe nogi pomachały! ')

def main(args=None):
    rclpy.init(args=args)

    dance_node = SimpleHexapodDance()
    
    try:
        # Inicjalizacja
        dance_node.wait_sim_time(2.0)
        
        # Wykonaj taniec
        dance_node.execute_simple_dance()
        
        # Chwila na zakończenie
        dance_node.wait_sim_time(1.0)
        
    except KeyboardInterrupt:
        print("\n Taniec przerwany")
    
    dance_node.destroy_node()
    rclpy.shutdown()
    print(" Koniec programu")

if __name__ == '__main__':
    main()