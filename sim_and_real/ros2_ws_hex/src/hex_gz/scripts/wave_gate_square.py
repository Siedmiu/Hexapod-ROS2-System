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

def trajektoria_prostokatna(start, cel, h, liczba_punktow):
    liczba_punktow += 3
    start_gora = start + np.array([0, 0, h])
    cel_gora = cel + np.array([0, 0, h])

    etap1 = np.linspace(start, start_gora, liczba_punktow // 3)
    etap2 = np.linspace(start_gora, cel_gora, liczba_punktow // 3)
    etap3 = np.linspace(cel_gora, cel, liczba_punktow - len(etap1) - len(etap2))

    punkty = np.concatenate([etap1[1:], etap2[1:], etap3[1:]], axis=0)
    return punkty

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


#w1 i w2 muszą mieć wspólnego x. kwadrat działa tylko do wave'a, jak chcemy inne to trzeba zmienić int(ilosc_punktow*2.5
def kwadrat(w1, w2, h, ilosc_punktow):
    punkty = []
    odleglosc = np.linalg.norm(w1 - w2)

    punkty_ruchu_y  = np.linspace(odleglosc * (ilosc_punktow - 1) / ilosc_punktow, 0, int(ilosc_punktow*2.5)-1)   
    punkty = [[w1[0], punkty_ruchu_y[i], w1[2] + h] for i in range((int(ilosc_punktow_na_krzywych*2.5)-1))]
    punkty.append(w2 + np.array([0,0,h]))
    return punkty


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

h = 0.15
r = 0.1
ilosc_punktow_na_krzywych = 20

punkty_etap1_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, 10000, 0)
punkty_etap2_ruchu_y = np.linspace(r * (ilosc_punktow_na_krzywych - 1) / ilosc_punktow_na_krzywych, 0, int(ilosc_punktow_na_krzywych*2.5))
punkty_etap2_ruchu = [[0, punkty_etap2_ruchu_y[i], 0] for i in range(int(ilosc_punktow_na_krzywych*2.5))]
punkty_etap3_ruchu_y = np.linspace(-r / ilosc_punktow_na_krzywych, -r, int(ilosc_punktow_na_krzywych*2.5))
punkty_etap3_ruchu = [[0, punkty_etap3_ruchu_y[i], 0] for i in range(int(ilosc_punktow_na_krzywych*2.5))]
punkty_etap4_ruchu = znajdz_punkty_kwadratowe(2 * r, h, ilosc_punktow_na_krzywych, 20000, -r)
punkty_etap5_ruchu = znajdz_punkty_kwadratowe(r, h / 2, ilosc_punktow_na_krzywych, 10000, -r)



print("stopa spoczynkowa")
print(stopa_spoczynkowa)

print("punkty_etap1_ruchu:")
print(punkty_etap1_ruchu)
print("punkty_etap2_ruchu")
print(punkty_etap2_ruchu)
print("punkty_etap3_ruchu")
print(punkty_etap3_ruchu)
print("punkty_etap4_ruchu")
print(punkty_etap4_ruchu)
print("punkty_etap5_ruchu")
print(punkty_etap5_ruchu)

cały_cykl = np.concatenate([punkty_etap2_ruchu, punkty_etap3_ruchu, punkty_etap4_ruchu])

fragmenty = np.array_split(cały_cykl, 6)
print(len(fragmenty[0]))

tył_1 = fragmenty[0]
tył_2 = fragmenty[1]
tył_3 = fragmenty[2]
tył_4 = fragmenty[3]
tył_5 = fragmenty[4]
czesc_z_parabola = fragmenty[5]

OPOZNIENIE_POSTOJU = 10

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


ilosc_cykli = 3

for _ in range(ilosc_cykli):
    cykl_nogi_1 = np.concatenate([cykl_nogi_1, tył_3_rozszerzony, tył_4_rozszerzony, tył_5_rozszerzony, czesc_z_parabola_rozszerzony, tył_1_rozszerzony, tył_2_rozszerzony])
    cykl_nogi_2 = np.concatenate([cykl_nogi_2, tył_2_rozszerzony, tył_3_rozszerzony, tył_4_rozszerzony, tył_5_rozszerzony, czesc_z_parabola_rozszerzony, tył_1_rozszerzony])
    cykl_nogi_3 = np.concatenate([cykl_nogi_3, tył_1_rozszerzony, tył_2_rozszerzony, tył_3_rozszerzony, tył_4_rozszerzony, tył_5_rozszerzony, czesc_z_parabola_rozszerzony])
    cykl_nogi_4 = np.concatenate([cykl_nogi_4, czesc_z_parabola_rozszerzony, tył_1_rozszerzony, tył_2_rozszerzony, tył_3_rozszerzony, tył_4_rozszerzony, tył_5_rozszerzony])
    cykl_nogi_5 = np.concatenate([cykl_nogi_5, tył_5_rozszerzony, czesc_z_parabola_rozszerzony, tył_1_rozszerzony, tył_2_rozszerzony, tył_3_rozszerzony, tył_4_rozszerzony])
    cykl_nogi_6 = np.concatenate([cykl_nogi_6, tył_4_rozszerzony, tył_5_rozszerzony, czesc_z_parabola_rozszerzony, tył_1_rozszerzony, tył_2_rozszerzony, tył_3_rozszerzony])


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


print("noga1: ", len(cykl_nogi_1))
print("noga2: ", len(cykl_nogi_2))
print("noga3: ", len(cykl_nogi_3))
print("noga4: ", len(cykl_nogi_4))
print("noga5: ", len(cykl_nogi_5))
print("noga6: ", len(cykl_nogi_6))

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

polozenia_stop_podczas_cyklu = np.array([ # polozenie_stop jest wzgledem ukladu nogi, gdzie przyczep do tulowia to punkt 0,0,0
    [[
        stopa_spoczynkowa[0] + cykle_nog[j][i][0],
        stopa_spoczynkowa[1] + cykle_nog[j][i][1],
        stopa_spoczynkowa[2] + cykle_nog[j][i][2]
    ]
    for i in range(len(cykl_nogi_1))]
    for j in range(6)
])


#wychyly podawane odpowiednio dla 1 2 i 3 przegubu w radianach
wychyly_serw_podczas_ruchu = np.array([
[katy_serw(polozenia_stop_podczas_cyklu[j][i], l1, l2, l3)
    for i in range(len(cykl_nogi_1))]
    for j in range(6)
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
    
    def execute_sequence(self, start_step=0, end_step=None, step_duration=0.2):
        """
        Wykonanie sekwencji ruchów dla wszystkich nóg równocześnie
        używając wartości z tablicy wychyly_serw_podczas_ruchu
        """
        self.get_logger().info('Rozpoczynam sekwencję ruchów dla wszystkich nóg')
        
        # Jeśli nie podano end_step, użyj całej tablicy
        if end_step is None:
            end_step = len(wychyly_serw_podczas_ruchu[0])
        
        # Przejście do pozycji początkowej (pierwszy punkt w tablicy)
        self.send_trajectory_to_all_legs_at_step(start_step, duration_sec=0.2)
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
