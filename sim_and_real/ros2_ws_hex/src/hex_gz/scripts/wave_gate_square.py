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
import argparse


matplotlib.use('TkAgg')

def katy_serw(P3, l1, l2, l3):
    # wyznaczenie katow potrzebnych do osiagniecia przez stope punktu docelowego
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


def polozenie_przegub_1(l1, alfa1, przyczep):
    return np.array([l1 * np.cos(alfa1) + przyczep[0], l1 * np.sin(alfa1) + przyczep[1], przyczep[2]])

def polozenie_przegub_2(l1, l2, alfa1, alfa2, przyczep):
    return polozenie_przegub_1(l1, alfa1, przyczep) + np.array(
        [l2 * np.cos(alfa1) * np.cos(alfa2), l2 * np.sin(alfa1) * np.cos(alfa2), l2 * np.sin(alfa2)])

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
        required_r = target_distance / (2 * cycles)
        
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

stopa_spoczynkowa = [x_start, 0, z_start]

wysokosc_start = -stopa_spoczynkowa[2]

nachylenia_nog_do_bokow_platformy_pajaka = np.array([
    np.deg2rad(45), 0, np.deg2rad(-45), np.deg2rad(180 + 45), np.deg2rad(180), np.deg2rad(180 - 45)
])

target_distance = 1
optimal_r, optimal_cycles = calculate_optimal_r_and_cycles(target_distance, l3)

parser = argparse.ArgumentParser()
parser.add_argument('--back', action='store_true', help='if true go back')
args = parser.parse_args()

h = 0.1
r = -optimal_r if args.back else optimal_r

ilosc_punktow_na_krzywych = 20

# =============================================================================
# TRAJECTORY OF LEGS
# =============================================================================

punkty_etap1_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, 10000, 0)
punkty_etap2_ruchu_y = np.linspace(r * (ilosc_punktow_na_krzywych - 1) / ilosc_punktow_na_krzywych, 0, int(ilosc_punktow_na_krzywych*2.5))
punkty_etap2_ruchu = [[0, punkty_etap2_ruchu_y[i], 0] for i in range(int(ilosc_punktow_na_krzywych*2.5))]
punkty_etap3_ruchu_y = np.linspace(-r / ilosc_punktow_na_krzywych, -r, int(ilosc_punktow_na_krzywych*2.5))
punkty_etap3_ruchu = [[0, punkty_etap3_ruchu_y[i], 0] for i in range(int(ilosc_punktow_na_krzywych*2.5))]
punkty_etap4_ruchu = znajdz_punkty_kwadratowe(2 * r, h, ilosc_punktow_na_krzywych, 20000, -r)
punkty_etap5_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, 10000, -r)

cały_cykl = np.concatenate([punkty_etap2_ruchu, punkty_etap3_ruchu, punkty_etap4_ruchu])

fragmenty = np.array_split(cały_cykl, 6)

tył_1 = fragmenty[0]
tył_2 = fragmenty[1]
tył_3 = fragmenty[2]
tył_4 = fragmenty[3]
tył_5 = fragmenty[4]
czesc_z_parabola = fragmenty[5]


#a delay for each leg, causes all legs to stop in their current positions when any leg is placed on the ground
OPOZNIENIE_POSTOJU = 10

#adjusting to initial positions

tył_1_rozszerzony = tył_1.copy()
for _ in range(OPOZNIENIE_POSTOJU):
    tył_1_rozszerzony = np.concatenate([tył_1_rozszerzony, [tył_1[-1]]])

tył_2_rozszerzony = tył_2.copy()
for _ in range(OPOZNIENIE_POSTOJU):
    tył_2_rozszerzony = np.concatenate([tył_2_rozszerzony, [tył_2[-1]]])

tył_3_rozszerzony = tył_3.copy()
for _ in range(OPOZNIENIE_POSTOJU):
    tył_3_rozszerzony = np.concatenate([tył_3_rozszerzony, [tył_3[-1]]])

tył_4_rozszerzony = tył_4.copy()
for _ in range(OPOZNIENIE_POSTOJU):
    tył_4_rozszerzony = np.concatenate([tył_4_rozszerzony, [tył_4[-1]]])

tył_5_rozszerzony = tył_5.copy()
for _ in range(OPOZNIENIE_POSTOJU):
    tył_5_rozszerzony = np.concatenate([tył_5_rozszerzony, [tył_5[-1]]])

czesc_z_parabola_rozszerzony = czesc_z_parabola.copy()
for _ in range(OPOZNIENIE_POSTOJU):
    czesc_z_parabola_rozszerzony = np.concatenate([czesc_z_parabola_rozszerzony, [czesc_z_parabola[-1]]])

odleglosc_srodek_tyl1 = np.linalg.norm(tył_1[-1] - punkty_etap1_ruchu[0])
odleglosc_srodek_tyl2 = np.linalg.norm(tył_2[-1] - punkty_etap1_ruchu[0])
odleglosc_srodek_tyl3 = np.linalg.norm(tył_3[-1] - punkty_etap1_ruchu[0])
odleglosc_srodek_tyl4 = np.linalg.norm(tył_4[-1] - punkty_etap1_ruchu[0])
odleglosc_srodek_tyl5 = np.linalg.norm(tył_5[-1] - punkty_etap1_ruchu[0])

dlugośc_malego_kroku = 10

pierwszy_krok_1_nogi = znajdz_punkty_kwadratowe(odleglosc_srodek_tyl2, h/2, dlugośc_malego_kroku, 10000, 0)
for i in range(OPOZNIENIE_POSTOJU):
    pierwszy_krok_1_nogi = np.concatenate([pierwszy_krok_1_nogi, [pierwszy_krok_1_nogi[-1]]])

template_rozszerzony = np.array([punkty_etap1_ruchu[0] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
pierwszy_krok_2_nogi = template_rozszerzony.copy()
pierwszy_krok_3_nogi = template_rozszerzony.copy()
pierwszy_krok_4_nogi = template_rozszerzony.copy()
pierwszy_krok_5_nogi = template_rozszerzony.copy()
pierwszy_krok_6_nogi = template_rozszerzony.copy()


drugi_krok_2_nogi = znajdz_punkty_kwadratowe(odleglosc_srodek_tyl1, h/2, dlugośc_malego_kroku, 10000, 0)
for i in range(OPOZNIENIE_POSTOJU):
    drugi_krok_2_nogi = np.concatenate([drugi_krok_2_nogi, [drugi_krok_2_nogi[-1]]])

drugi_krok_1_nogi = np.array([tył_2[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])

drugi_krok_3_nogi = template_rozszerzony.copy()
drugi_krok_4_nogi = template_rozszerzony.copy()
drugi_krok_5_nogi = template_rozszerzony.copy()
drugi_krok_6_nogi = template_rozszerzony.copy()


trzeci_krok_1_nogi = np.array([tył_2[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
trzeci_krok_2_nogi = np.array([tył_1[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
trzeci_krok_3_nogi = znajdz_punkty_kwadratowe(r, h / 2, dlugośc_malego_kroku, 10000, 0)
for i in range(OPOZNIENIE_POSTOJU):
    trzeci_krok_3_nogi = np.concatenate([trzeci_krok_3_nogi, [trzeci_krok_3_nogi[-1]]])
trzeci_krok_4_nogi = template_rozszerzony.copy()
trzeci_krok_5_nogi = template_rozszerzony.copy()
trzeci_krok_6_nogi = template_rozszerzony.copy()


czwarty_krok_1_nogi = np.array([tył_2[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
czwarty_krok_2_nogi = np.array([tył_1[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
czwarty_krok_3_nogi = np.array([punkty_etap4_ruchu[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
czwarty_krok_4_nogi = znajdz_punkty_kwadratowe(-r, h/2, dlugośc_malego_kroku, 1000, 0)
for i in range(OPOZNIENIE_POSTOJU):
    czwarty_krok_4_nogi = np.concatenate([czwarty_krok_4_nogi, [czwarty_krok_4_nogi[-1]]])
czwarty_krok_5_nogi = template_rozszerzony.copy()
czwarty_krok_6_nogi = template_rozszerzony.copy()


piaty_krok_1_nogi = np.array([tył_2[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
piaty_krok_2_nogi = np.array([tył_1[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
piaty_krok_3_nogi = np.array([punkty_etap4_ruchu[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
piaty_krok_4_nogi = np.array([tył_5[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
piaty_krok_5_nogi = znajdz_punkty_kwadratowe(-odleglosc_srodek_tyl4, h/2, dlugośc_malego_kroku, 1000, 0)
for i in range(OPOZNIENIE_POSTOJU):
    piaty_krok_5_nogi = np.concatenate([piaty_krok_5_nogi, [piaty_krok_5_nogi[-1]]])
piaty_krok_6_nogi = template_rozszerzony.copy()


szosty_krok_6_nogi = znajdz_punkty_kwadratowe(-odleglosc_srodek_tyl3, h/2, dlugośc_malego_kroku, 1000, 0)
for i in range(OPOZNIENIE_POSTOJU):
    szosty_krok_6_nogi = np.concatenate([szosty_krok_6_nogi, [szosty_krok_6_nogi[-1]]])
szosty_krok_1_nogi = np.array([tył_2[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
szosty_krok_2_nogi = np.array([tył_1[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
szosty_krok_3_nogi = np.array([punkty_etap4_ruchu[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
szosty_krok_4_nogi = np.array([tył_5[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
szosty_krok_5_nogi = np.array([tył_4[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])




cykl_nogi_1 = np.concatenate([pierwszy_krok_1_nogi, drugi_krok_1_nogi, trzeci_krok_1_nogi, czwarty_krok_1_nogi, piaty_krok_1_nogi, szosty_krok_1_nogi])
cykl_nogi_2 = np.concatenate([pierwszy_krok_2_nogi, drugi_krok_2_nogi, trzeci_krok_2_nogi, czwarty_krok_2_nogi, piaty_krok_2_nogi, szosty_krok_2_nogi])
cykl_nogi_3 = np.concatenate([pierwszy_krok_3_nogi, drugi_krok_3_nogi, trzeci_krok_3_nogi, czwarty_krok_3_nogi, piaty_krok_3_nogi, szosty_krok_3_nogi])
cykl_nogi_4 = np.concatenate([pierwszy_krok_4_nogi, drugi_krok_4_nogi, trzeci_krok_4_nogi, czwarty_krok_4_nogi, piaty_krok_4_nogi, szosty_krok_4_nogi])
cykl_nogi_5 = np.concatenate([pierwszy_krok_5_nogi, drugi_krok_5_nogi, trzeci_krok_5_nogi, czwarty_krok_5_nogi, piaty_krok_5_nogi, szosty_krok_5_nogi])
cykl_nogi_6 = np.concatenate([pierwszy_krok_6_nogi, drugi_krok_6_nogi, trzeci_krok_6_nogi, czwarty_krok_6_nogi, piaty_krok_6_nogi, szosty_krok_6_nogi])


ilosc_cykli = optimal_cycles

#MAIN LOOP

for _ in range(ilosc_cykli):
    cykl_nogi_1 = np.concatenate([cykl_nogi_1, tył_3_rozszerzony, tył_4_rozszerzony, tył_5_rozszerzony, czesc_z_parabola_rozszerzony, tył_1_rozszerzony, tył_2_rozszerzony])
    cykl_nogi_2 = np.concatenate([cykl_nogi_2, tył_2_rozszerzony, tył_3_rozszerzony, tył_4_rozszerzony, tył_5_rozszerzony, czesc_z_parabola_rozszerzony, tył_1_rozszerzony])
    cykl_nogi_3 = np.concatenate([cykl_nogi_3, tył_1_rozszerzony, tył_2_rozszerzony, tył_3_rozszerzony, tył_4_rozszerzony, tył_5_rozszerzony, czesc_z_parabola_rozszerzony])
    cykl_nogi_4 = np.concatenate([cykl_nogi_4, czesc_z_parabola_rozszerzony, tył_1_rozszerzony, tył_2_rozszerzony, tył_3_rozszerzony, tył_4_rozszerzony, tył_5_rozszerzony])
    cykl_nogi_5 = np.concatenate([cykl_nogi_5, tył_5_rozszerzony, czesc_z_parabola_rozszerzony, tył_1_rozszerzony, tył_2_rozszerzony, tył_3_rozszerzony, tył_4_rozszerzony])
    cykl_nogi_6 = np.concatenate([cykl_nogi_6, tył_4_rozszerzony, tył_5_rozszerzony, czesc_z_parabola_rozszerzony, tył_1_rozszerzony, tył_2_rozszerzony, tył_3_rozszerzony])

#RETURN TO HOME POSITIONS

pierwszy_od_konca_krok_1_nogi = pierwszy_krok_1_nogi[::-1]
pierwszy_od_konca_krok_2_nogi = np.array([tył_1[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
pierwszy_od_konca_krok_3_nogi = np.array([punkty_etap4_ruchu[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
pierwszy_od_konca_krok_4_nogi = np.array([tył_5[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
pierwszy_od_konca_krok_5_nogi = np.array([tył_4[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
pierwszy_od_konca_krok_6_nogi = np.array([tył_3[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])

drugi_od_konca_krok_1_nogi = template_rozszerzony.copy()
drugi_od_konca_krok_2_nogi = drugi_krok_2_nogi[::-1]
drugi_od_konca_krok_3_nogi = np.array([punkty_etap4_ruchu[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
drugi_od_konca_krok_4_nogi = np.array([tył_5[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
drugi_od_konca_krok_5_nogi = np.array([tył_4[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
drugi_od_konca_krok_6_nogi = np.array([tył_3[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])


trzeci_od_konca_krok_1_nogi = template_rozszerzony.copy()
trzeci_od_konca_krok_2_nogi = template_rozszerzony.copy()
trzeci_od_konca_krok_3_nogi = trzeci_krok_3_nogi[::-1]
trzeci_od_konca_krok_4_nogi = np.array([tył_5[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
trzeci_od_konca_krok_5_nogi = np.array([tył_4[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
trzeci_od_konca_krok_6_nogi = np.array([tył_3[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])

czwarty_od_konca_krok_1_nogi = template_rozszerzony.copy()
czwarty_od_konca_krok_2_nogi = template_rozszerzony.copy()
czwarty_od_konca_krok_3_nogi = template_rozszerzony.copy()
czwarty_od_konca_krok_4_nogi = czwarty_krok_4_nogi[::-1]
czwarty_od_konca_krok_5_nogi = np.array([tył_4[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])
czwarty_od_konca_krok_6_nogi = np.array([tył_3[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])


piaty_od_konca_krok_1_nogi = template_rozszerzony.copy()
piaty_od_konca_krok_2_nogi = template_rozszerzony.copy()
piaty_od_konca_krok_3_nogi = template_rozszerzony.copy()
piaty_od_konca_krok_4_nogi = template_rozszerzony.copy()
piaty_od_konca_krok_5_nogi = piaty_krok_5_nogi[::-1]
piaty_od_konca_krok_6_nogi = np.array([tył_3[-1] for _ in range(dlugośc_malego_kroku + OPOZNIENIE_POSTOJU)])


szosty_od_konca_krok_1_nogi = template_rozszerzony.copy()
szosty_od_konca_krok_2_nogi = template_rozszerzony.copy()
szosty_od_konca_krok_3_nogi = template_rozszerzony.copy()
szosty_od_konca_krok_4_nogi = template_rozszerzony.copy()
szosty_od_konca_krok_5_nogi = template_rozszerzony.copy()
szosty_od_konca_krok_6_nogi = szosty_krok_6_nogi[::-1]


cykl_nogi_1 = np.concatenate([cykl_nogi_1, pierwszy_od_konca_krok_1_nogi, drugi_od_konca_krok_1_nogi, trzeci_od_konca_krok_1_nogi, czwarty_od_konca_krok_1_nogi, piaty_od_konca_krok_1_nogi, szosty_od_konca_krok_1_nogi])
cykl_nogi_2 = np.concatenate([cykl_nogi_2, pierwszy_od_konca_krok_2_nogi, drugi_od_konca_krok_2_nogi, trzeci_od_konca_krok_2_nogi, czwarty_od_konca_krok_2_nogi, piaty_od_konca_krok_2_nogi, szosty_od_konca_krok_2_nogi])
cykl_nogi_3 = np.concatenate([cykl_nogi_3, pierwszy_od_konca_krok_3_nogi, drugi_od_konca_krok_3_nogi, trzeci_od_konca_krok_3_nogi, czwarty_od_konca_krok_3_nogi, piaty_od_konca_krok_3_nogi, szosty_od_konca_krok_3_nogi])
cykl_nogi_4 = np.concatenate([cykl_nogi_4, pierwszy_od_konca_krok_4_nogi, drugi_od_konca_krok_4_nogi, trzeci_od_konca_krok_4_nogi, czwarty_od_konca_krok_4_nogi, piaty_od_konca_krok_4_nogi, szosty_od_konca_krok_4_nogi])
cykl_nogi_5 = np.concatenate([cykl_nogi_5, pierwszy_od_konca_krok_5_nogi, drugi_od_konca_krok_5_nogi, trzeci_od_konca_krok_5_nogi, czwarty_od_konca_krok_5_nogi, piaty_od_konca_krok_5_nogi, szosty_od_konca_krok_5_nogi])
cykl_nogi_6 = np.concatenate([cykl_nogi_6, pierwszy_od_konca_krok_6_nogi, drugi_od_konca_krok_6_nogi, trzeci_od_konca_krok_6_nogi, czwarty_od_konca_krok_6_nogi, piaty_od_konca_krok_6_nogi, szosty_od_konca_krok_6_nogi])

# Update the cycle array to use the new unified cycle
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
    ] if j == 4 else [
        [cykl_nogi_6[i][1] * np.sin(nachylenia_nog_do_bokow_platformy_pajaka[j]),
        cykl_nogi_6[i][1] * np.cos(nachylenia_nog_do_bokow_platformy_pajaka[j]),
        cykl_nogi_6[i][2]]
        for i in range(len(cykl_nogi_6))
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
    for i in range(len(cykl_nogi_1))]
    for j in range(6)
])

#results given for 1st 2 and 3rd joint in radians respectively
wychyly_serw_podczas_ruchu = np.array([
[katy_serw(polozenia_stop_podczas_cyklu[j][i], l1, l2, l3)
    for i in range(len(cykl_nogi_1))]
    for j in range(6)
])

class LegSequencePlayer(Node):
    def __init__(self):
        super().__init__('leg_sequence_player')
        self.get_logger().info('Initialisation...')
        
        # Contact status storage for each leg
        self.contact_status = {
            1: False, 2: False, 3: False, 
            4: False, 5: False, 6: False
        }
        
        # Leg controlers
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
        Sends the trajectory to the controllers of all legs at the same time
        using the values from the swing_serv_motion array for the step
        """
        self.get_logger().info(f'Wysyłam trajektorię dla kroku {step_index}')
        
        # Check if index is correct
        if step_index >= len(wychyly_serw_podczas_ruchu[0]):
            self.get_logger().error(f'Index step {step_index} out of range!')
            return False
        
        # Time of movement
        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        # Prepare nad send trajectory to each leg
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
            
            # Time of movement
            point.time_from_start = duration
            
            trajectory.points.append(point)
            
            # Send trajectory
            self.trajectory_publishers[leg_num].publish(trajectory)
        
        self.get_logger().info('Trajectory to each leg sent')
        return True
    
    def execute_sequence(self, start_step=0, end_step=None, step_duration=0.2):
        
        """
        Perform a sequence of movements for all legs simultaneously
        using the values from the swing_serv_movement array
        """
        
        if end_step is None:
            end_step = len(wychyly_serw_podczas_ruchu[0])
        
        # First position
        self.send_trajectory_to_all_legs_at_step(start_step, duration_sec=0.2)
        self.get_logger().info('Waiting for first movement...')
        time.sleep(step_duration)
        
        # Movement sequence
        for step in range(start_step + 1, end_step):
            self.send_trajectory_to_all_legs_at_step(step, duration_sec=step_duration)
            time.sleep(step_duration)
        
        self.get_logger().info('Sekwencja zakończona')

    def execute_sequence_with_contact_monitoring(self, start_step=0, end_step=None, step_duration=0.2):
        """
        Execute sequence with contact status monitoring for safer walking
        """
        
        if end_step is None:
            end_step = len(wychyly_serw_podczas_ruchu[0])
        
        # Initial position
        self.send_trajectory_to_all_legs_at_step(start_step, duration_sec=0.2)
        time.sleep(3.0)
        
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
                if int((time.time() - start_time) * 20) % 4 == 0:  # Every 0.2 seconds
                    contact_info = self.get_all_contact_status()
                    self.get_logger().info(f'Krok {step}, Kontakt: {contact_info}')
        
        self.get_logger().info('Sekwencja z monitorowaniem zakończona')


def main(args=None):
    rclpy.init(args=args)
    
    # Utworzenie węzła
    node = LegSequencePlayer()
    
    try:
        print("Initialisation...")
        time.sleep(0.5)
        
        node.execute_sequence()
        
        time.sleep(0.5)
        
    except KeyboardInterrupt:
        pass
    
    # Cleaning
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()