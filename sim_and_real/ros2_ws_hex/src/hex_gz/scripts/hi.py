#!/usr/bin/env python3

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

spocz_1 = 0
spocz_2 = np.deg2rad(10)
spocz_3 = np.deg2rad(80)

ruch_1_1 = 0
ruch_1_2 = np.deg2rad(-55)
ruch_1_3 = np.deg2rad(-30)

ruch_2_1 = np.deg2rad(-10)
ruch_2_2 = np.deg2rad(-55)
ruch_2_3 = np.deg2rad(-30)

ruch_3_1 = np.deg2rad(10)
ruch_3_2 = np.deg2rad(-55)
ruch_3_3 = np.deg2rad(-30)

#wychyly podawane odpowiednio dla 1 2 i 3 przegubu w radianach
wychyly_serw_podczas_ruchu = np.array([
[[spocz_1, spocz_2, spocz_3], [ruch_1_1, ruch_1_2, ruch_1_3], [ruch_2_1, ruch_2_2, ruch_2_3], [ruch_3_1, ruch_3_2, ruch_3_3], [ruch_2_1, ruch_2_2, ruch_2_3], [ruch_3_1, ruch_3_2, ruch_3_3], [spocz_1, spocz_2, spocz_3]],
[[spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3],[spocz_1, spocz_2, spocz_3]],
[[spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3],[spocz_1, spocz_2, spocz_3]],
[[spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3],[spocz_1, spocz_2, spocz_3]],
[[spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3],[spocz_1, spocz_2, spocz_3]],
[[spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3],[spocz_1, spocz_2, spocz_3]]
])

class LegSequencePlayer(Node):
    def __init__(self):
        super().__init__('leg_sequence_player')
        self.get_logger().info('Inicjalizacja węzła do sekwencji ruchów')
        
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
        """
        Wysyła trajektorię do kontrolerów wszystkich nóg jednocześnie
        używając wartości z tablicy wychyly_serw_podczas_ruchu dla danego kroku
        """
        self.get_logger().info(f'Wysyłam trajektorię dla kroku {step_index}')
        
        # Sprawdź, czy indeks jest prawidłowy
        if step_index >= len(wychyly_serw_podczas_ruchu[0]):
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
            joint_values = wychyly_serw_podczas_ruchu[leg_num-1][step_index]
            
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
    
    def execute_sequence(self, start_step=0, end_step=None, step_duration=0.5):
        """
        Wykonanie sekwencji ruchów dla wszystkich nóg równocześnie
        używając wartości z tablicy wychyly_serw_podczas_ruchu
        """
        self.get_logger().info('Rozpoczynam sekwencję ruchów dla wszystkich nóg')
        
        # Jeśli nie podano end_step, użyj całej tablicy
        if end_step is None:
            end_step = len(wychyly_serw_podczas_ruchu[0])
        
        # Przejście do pozycji początkowej (pierwszy punkt w tablicy)
        self.send_trajectory_to_all_legs_at_step(start_step, duration_sec=3.0)
        self.get_logger().info('Oczekiwanie na wykonanie początkowego ruchu...')
        time.sleep(3.0)
        
        # Wykonanie sekwencji ruchów
        for step in range(start_step + 1, end_step):
            self.send_trajectory_to_all_legs_at_step(step, duration_sec=step_duration)
            self.get_logger().info(f'Wykonano krok {step}, oczekiwanie {step_duration}s...')
            time.sleep(step_duration)
        
        self.get_logger().info('Sekwencja zakończona')


def main(args=None):
    rclpy.init(args=args)
    
    # Utworzenie węzła
    node = LegSequencePlayer()
    
    try:
        # Krótkie oczekiwanie na inicjalizację
        print("Inicjalizacja... Poczekaj 2 sekundy.")
        time.sleep(2.0)
        
        # Wykonanie sekwencji
        print("Rozpoczynam sekwencję")
        node.execute_sequence()
        
        # Utrzymanie węzła aktywnego przez chwilę
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        pass
    
    # Sprzątanie
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()