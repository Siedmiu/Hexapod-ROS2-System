#!/usr/bin/env python3

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
<<<<<<< HEAD
=======
from mpl_toolkits.mplot3d import Axes3D
>>>>>>> b454fa09f39016cd29204c8bb967c07b23d315c7


matplotlib.use('TkAgg')

def katy_serw(P3, l1, l2, l3):
    # wyznaczenie katow potrzebnych do osiagniecia przez stope punktu docelowego
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
<<<<<<< HEAD
    punkty_w_dol = ilosc_punktow_na_krzywych - punkty_w_gore - punkty_do_przodu  # reszta na ruch w dół
=======
    punkty_w_dol = ilosc_punktow_na_krzywej - punkty_w_gore - punkty_do_przodu  # reszta na ruch w dół
>>>>>>> b454fa09f39016cd29204c8bb967c07b23d315c7
    
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

l1 = 0.17995 - 0.12184
l2 = 0.30075 - 0.17995
l3 = 0.50975 - 0.30075

# Położenie punktu spoczynku od przyczepu nogi wyznaczone na bazie katow przgubow podczas spoczynku
# WAZNE !!! jest to polozenie stopy w ukladzie punktu zaczepienia stopy a nie ukladu XYZ
# w ktorym X1 to prostopadła prosta do boku platformy do ktorej noga jest zaczepiona i rosnie w kierunku od hexapoda
# Y1 to os pokrywajaca sie z bokiem platformy do ktorego jest przyczepiona noga i rosnie w kierunku przodu hexapoda
# Z1 pokrywa sie z osia Z ukladu XYZ

# zalozone katy spoczynkowe przegubow
alfa_1 = 0
alfa_2 = np.radians(10)
alfa_3 = np.radians(80)

x_start = l1 + l2 * np.cos(alfa_2) + l3 * np.sin(np.deg2rad(90) - alfa_2 - alfa_3)  # poczatkowe wychylenie nogi pajaka w osi x
z_start = -(l2*np.sin(alfa_2) + l3 * np.cos(np.deg2rad(90) - alfa_2 - alfa_3))  # poczatkowy z

stopa_spoczynkowa = [x_start, 0, z_start]

wysokosc_start = -stopa_spoczynkowa[2]

nachylenia_nog_do_bokow_platformy_pajaka = np.array([
    np.deg2rad(45), 0, np.deg2rad(-45), np.deg2rad(180 + 45), np.deg2rad(180), np.deg2rad(180 - 45)
])

target_distance = 1
optimal_r, optimal_cycles = calculate_optimal_r_and_cycles(target_distance, l3)
print("optimal r:", optimal_r)
print("optimal cycles:", optimal_cycles)

h = 0.1
r = optimal_r

ilosc_punktow_na_krzywych = 20

polozenie_stop = np.array([stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa])

def step_up(p1, delta_h):
    """
    Generuje ruch w górę o 1 krok
    """
    return p1 + np.array([0, 0, delta_h])


def step_forward(p1, delta_r):
    """
    Generuje ruch do przodu o 1 krok
    """
    return p1 + np.array([0, delta_r, 0])

def step_backward(p1, delta_r):
    """
    Generuje ruch do przodu o 1 krok
    """
    return p1 + np.array([0, -delta_r, 0])


def step_down(p1, delta_h):
    """
    Generuje ruch w dół o 1 krok
    """
    return p1 + np.array([0, 0, -delta_h])



def counting_point_for_each_leg(p1, nachylenie):
    """
    Oblicza punkt dla każdej nogi na podstawie kąta nachylenia
    """
    p2 = [0, p1[1], p1[2]]
    p2 = [-p2[1]*np.sin(nachylenie), p2[1]*np.cos(nachylenie), p1[2]]
    p2 = [p1[0] + p2[0], p2[1], p2[2]]
    p2 = np.array(p2)
    return p2

""" 

1 nogi idą do tyłu/góry w momencie gdy wszystkie stoją na ziemii
2 jak noga odpaliła czujnik to czeka na wszytskie na swoich miejscach lub czujnikach
3 jak noga doszła do stanu docelonego a któraś jeszcze nie odpaliła czujnika, to czeka aż odpali

START
nogi 1 3 5: góra --> przód --> dół (czeka aż wszystkie na miejscach 2 4 6 a 1 3 5 wcisniete)
nogi 2 4 6: po prostej do tylu (czeka aż wszystkie na miejscach 2 4 6 a 1 3 5 wcisniete)          

PĘTLA
nogi 1 3 5: po prostej do tyłu (czeka aż wszystkie na miejscach 1 3 5 a 2 4 6 wcisniete)         --> góra --> przód --> dół  (czeka aż wszystkie na miejscach 2 4 6 a 1 3 5 wcisniete)
nogi 2 4 6: góra--> przód --> dół (czeka aż wszystkie na miejscach 1 3 5 a 2 4 6 wcisniete)      --> po prostej do tyłu      (czeka aż wszystkie na miejscach 2 4 6 a 1 3 5 wcisniete) 

KONIEC
nogi 1 3 5: po ziemii do środka
nogi 2 4 6: góra --> przód do środka --> dół
"""

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
        
        # Listy stawów dla nóg
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


<<<<<<< HEAD
    def execute_single_move(self, leg_positions, duration_sec=0.5):
=======
    def execute_single_move(self, leg_positions, duration_sec=0.2):
>>>>>>> b454fa09f39016cd29204c8bb967c07b23d315c7
        """
        Wykonuje jeden ruch i zwraca status kontaktu wszystkich nóg
        
        Args:
            leg_positions: dict {leg_num: [x, y, z], ...} lub lista 6 pozycji
            duration_sec: czas trwania ruchu
        
        Returns:
            dict: {leg_num: contact_status, ...}
        """
        # Konwersja listy na dict jeśli potrzeba
        if isinstance(leg_positions, list) and len(leg_positions) == 6:
            leg_positions = {i+1: leg_positions[i] for i in range(6)}
        
        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        # Wyślij trajektorie do wszystkich nóg
        for leg_num in range(1, 7):
<<<<<<< HEAD
=======
            contact_info = self.get_contact_status(leg_num)
            self.get_logger().info(f"noga {leg_num} kontakt: {contact_info}")
>>>>>>> b454fa09f39016cd29204c8bb967c07b23d315c7
            if leg_num in leg_positions:
                try:
                    joint_angles = katy_serw(leg_positions[leg_num], l1, l2, l3)
                    print("joint_angles dla nogi", leg_num, ":", joint_angles)
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
                    self.get_logger().error(f'Błąd dla nogi {leg_num}: {e}')
<<<<<<< HEAD
        
=======

>>>>>>> b454fa09f39016cd29204c8bb967c07b23d315c7
        # Czekaj na zakończenie ruchu
        time.sleep(duration_sec + 0.1)
        
        # Sprawdź ostatni status kontaktu
        rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.contact_status.copy()


def main(args=None):
    rclpy.init(args=args)
    
    # Utworzenie węzła
    node = LegSequencePlayer()
    
<<<<<<< HEAD
=======
    trajektorie_nog = [[] for _ in range(6)]

>>>>>>> b454fa09f39016cd29204c8bb967c07b23d315c7
    try:
        # Krótkie oczekiwanie na inicjalizację
        print("Inicjalizacja... Poczekaj 2 sekundy.")
        time.sleep(2.0)
        
        # Wykonanie sekwencji
        print("Rozpoczynam sekwencję z monitorowaniem kontaktu")

<<<<<<< HEAD
        h = 0.1
        r = 0.08

        kroczek_gora = h/20
        kroczek_tyl = r/60
=======
        wysokosc_podnoszenia = 0.07
        dlugosc_kroku = 0.04

        h = wysokosc_podnoszenia
        r = dlugosc_kroku

        ilosc_punktow = 5

        kroczek_gora = h/ilosc_punktow
        kroczek_tyl = r/(ilosc_punktow*3)
>>>>>>> b454fa09f39016cd29204c8bb967c07b23d315c7

        leg_positions = [stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa, stopa_spoczynkowa]
        point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1]) for leg, pos in enumerate(leg_positions, start=1)]
        print("Początkowe pozycje nóg:", point_positions)
        
<<<<<<< HEAD

        while(h > 0):
            for leg in range(1, 7):
                if leg in [1, 3, 5]:
                    leg_positions[leg-1] = step_up(leg_positions[leg-1], kroczek_gora)
                else:
                    leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_gora)
=======
        

        while(h > 0):
            for leg in range(1, 7):
                if leg in [1]:
                    leg_positions[leg-1] = step_up(leg_positions[leg-1], kroczek_gora)
                else:
                    leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
>>>>>>> b454fa09f39016cd29204c8bb967c07b23d315c7
            point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1]) for leg, pos in enumerate(leg_positions, start=1)]
            print("h=",h)
            print("Pozycje nóg:", point_positions)
            node.execute_single_move(point_positions)
<<<<<<< HEAD
            h -= kroczek_gora
        
        
        # Utrzymanie węzła aktywnego przez chwilę
        time.sleep(2.0)
=======
            for i in range(0,6):
                trajektorie_nog[i].append(point_positions[i])
            h -= kroczek_gora

        h = wysokosc_podnoszenia
        gorny_kroczek_przod = r/ilosc_punktow

        while(r > 0):
            for leg in range(1, 7):
                if leg in [1]:
                    leg_positions[leg-1] = step_forward(leg_positions[leg-1], gorny_kroczek_przod)
                else:
                    leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
            point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1]) for leg, pos in enumerate(leg_positions, start=1)]
            print("r=",r)
            print("Pozycje nóg:", point_positions)
            node.execute_single_move(point_positions)
            for i in range(0,6):
                trajektorie_nog[i].append(point_positions[i])
            r -= gorny_kroczek_przod

        r = dlugosc_kroku    

        pozycja_docelowa_2 = step_backward(leg_positions[1], r/3)
        pozycja_docelowa_3 = step_backward(leg_positions[2], r/3)
        pozycja_docelowa_4 = step_backward(leg_positions[3], r/3)
        pozycja_docelowa_5 = step_backward(leg_positions[4], r/3)
        pozycja_docelowa_6 = step_backward(leg_positions[5], r/3)

        pozycje_docelowe = [0, pozycja_docelowa_2, pozycja_docelowa_4, pozycja_docelowa_4, pozycja_docelowa_5, pozycja_docelowa_6]
        
        ilosc_punktow_opadajacych = 30

        kroczek_dol = h/ilosc_punktow_opadajacych

        czy_noga_stala = [False, True, True, True, True, True]

        while(not czy_noga_stala[0] or not czy_noga_stala[2] or not czy_noga_stala[4]):
            for leg in range(1, 7):
                print(f"czy byl styk nogi {leg}: ", czy_noga_stala[leg - 1])
                if leg in [1]:
                    if not node.get_contact_status(leg):
                        if not czy_noga_stala[leg - 1]:
                            leg_positions[leg-1] = step_down(leg_positions[leg-1], kroczek_dol)
                            print(f"rusza {leg}")
                    else:
                        czy_noga_stala[leg-1] = True
                else:
                    if leg_positions[leg-1][1] > pozycje_docelowe[leg-1][1]:
                        leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)

            point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1]) for leg, pos in enumerate(leg_positions, start=1)]
            print("h=",h)
            print("Pozycje nóg:", point_positions)
            node.execute_single_move(point_positions)

            for i in range(0,6):
                trajektorie_nog[i].append(point_positions[i])
            h -= kroczek_dol


        """
        while(leg_positions[1][1] > pozycja_docelowa_2[1] or leg_positions[3][1] > pozycja_docelowa_4[1] or leg_positions[5][1] > pozycja_docelowa_6[1] or not node.get_contact_status(1) or not node.get_contact_status(3) or not node.get_contact_status(5)):
            print("jestem")

            for leg in [1, 3, 5]:
                if not node.get_contact_status(leg):
                    leg_positions[leg-1] = step_down(leg_positions[leg-1], kroczek_dol)
                    print(f"noga: {leg} conact status: {node.get_contact_status(leg)}")
            for leg in [2, 4, 6]:
                if leg_positions[leg-1][1] > pozycje_docelowe[leg-1][1]:
                    leg_positions[leg-1] = step_backward(leg_positions[leg-1], kroczek_tyl)
                else:
                    if leg_positions[leg-1][1] != pozycje_docelowe[leg-1][1]:
                        print("pozycja przed doprowadzeniem do końca:", leg_positions[leg-1])
                        leg_positions[leg-1] = pozycje_docelowe[leg-1]
                        print("pozycja po doprowadzeniem do końca:", leg_positions[leg-1])
                        
            point_positions = [counting_point_for_each_leg(pos, nachylenia_nog_do_bokow_platformy_pajaka[leg-1]) for leg, pos in enumerate(leg_positions, start=1)]
            node.execute_single_move(point_positions)
            print("war1", not node.get_contact_status(1))
            print("war2", not node.get_contact_status(3))
            print("war3", not node.get_contact_status(5))

            for i in range(0,6):
                trajektorie_nog[i].append(point_positions[i])
        """

        # Utrzymanie węzła aktywnego przez chwilę
        time.sleep(2.0)

    
        fig = plt.figure(figsize=(15, 10))

        for i in range(6):
            ax = fig.add_subplot(3, 2, i + 1, projection='3d')
            trajektoria = trajektorie_nog[i]
            x = [p[0] for p in trajektoria]
            y = [p[1] for p in trajektoria]
            z = [p[2] for p in trajektoria]
            ax.plot(x, y, z, marker='o')
            ax.set_title(f'Noga {i+1}')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.grid(True)

        plt.tight_layout()
        plt.show()
>>>>>>> b454fa09f39016cd29204c8bb967c07b23d315c7
        
    except KeyboardInterrupt:
        pass
    
    # Sprzątanie
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
<<<<<<< HEAD
    main()
=======
    main()
>>>>>>> b454fa09f39016cd29204c8bb967c07b23d315c7
