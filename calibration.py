import sys
import os
import time
import numpy as np

"""
Ce fichier permet de calibrer facilement et rapidement notre DDBoat
La matrice A et le bias b sont sauvegardés avec un np.save()
"""

# Assure-toi que les modules sont correctement importés
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

import imu9_driver_v2 as imudrv
import gps_driver_v2 as gpsdrv

# Création de l'objet IMU
imu = imudrv.Imu9IO()

### Calibration de la boussole ###
def average_measurements_mag(imu, n=10):
    """Prend la moyenne des n lectures magnétiques."""
    x_total, y_total, z_total = 0, 0, 0
    for _ in range(n):
        xmag, ymag, zmag = imu.read_mag_raw()
        x_total += xmag
        y_total += ymag
        z_total += zmag
        time.sleep(0.1)
    return np.array([x_total / n, y_total / n, z_total / n])  # Retourne un vecteur 3x1

def calib_mag():
    print("### Début de la calibration de la boussole ###")
    # Préparation pour les mesures
    print("Placez le bateau vers le Nord, puis appuyez sur Entrée.")
    input()
    x_nord = average_measurements_mag(imu)
    print("Mesure x_nord prise : {}".format(x_nord))

    print("Placez le bateau vers le Sud, puis appuyez sur Entrée.")
    input()
    x_sud = average_measurements_mag(imu)
    print("Mesure x_sud prise : {}".format(x_sud))

    print("Placez le bateau vers l'Ouest, puis appuyez sur Entrée.")
    input()
    x_west = average_measurements_mag(imu)
    print("Mesure x_west prise : {}".format(x_west))

    print("Placez le bateau vers le Haut, puis appuyez sur Entrée.")
    input()
    x_up = average_measurements_mag(imu)
    print("Mesure x_up prise : {}".format(x_up))

    # Calcul du biais b (vecteur 3x1)
    b = -0.5 * (x_nord + x_sud)
    print("Biais calculé : b = {}".format(b))

    # Calcul de B et I
    B = 46 * 10**(-6)  # T
    I = 64 * np.pi / 180  # rad

    # Calcul des y (vecteurs théoriques, chaque vecteur est 3x1)
    y_nord = np.array([B * np.cos(I), 0, -B * np.sin(I)])
    y_west = np.array([0, -B * np.cos(I), -B * np.sin(I)])
    y_up = np.array([-B * np.sin(I), 0, -B * np.cos(I)])

    # Construction de la matrice Y 3x3 avec y_nord, y_west, y_up comme colonnes
    Y = np.column_stack([y_nord, y_west, y_up])

    # Calcul de l'inverse de la matrice Y
    Y_inv = np.linalg.inv(Y)

    # Construction des vecteurs corrigés pour X
    x_nord_corr = x_nord + b
    x_west_corr = x_west + b
    x_up_corr = x_up + b

    # Construction de la matrice X 3x3 avec x_nord_corr, x_west_corr, x_up_corr comme colonnes
    X = np.column_stack([x_nord_corr, x_west_corr, x_up_corr])

    # Calcul de la matrice A (produit matriciel entre X et Y_inv)
    A = np.dot(X, Y_inv)
    print("Matrice A calculée : {}".format(A))

    return b, A

### Calibration de l'accéléromètre ###
def average_measurements_accel(imu, n=10):
    """Prend la moyenne des n lectures d'accéléromètre."""
    x_total, y_total, z_total = 0, 0, 0
    for _ in range(n):
        xaccel, yaccel, zaccel = imu.read_accel_raw()
        x_total += xaccel
        y_total += yaccel
        z_total += zaccel
        time.sleep(0.1)
    return np.array([x_total / n, y_total / n, z_total / n])  # Retourne un vecteur 3x1

def calib_accel():
    print("### Début de la calibration de l'accéléromètre ###")
    # Préparation pour les mesures
    print("Placez le x du bateau vers le haut, puis appuyez sur Entrée.")
    input()
    x_x = average_measurements_accel(imu)
    print("Mesure x_x prise : {}".format(x_x))

    print("Placez le z du bateau vers le haut, puis appuyez sur Entrée.")
    input()
    x_z = average_measurements_accel(imu)
    print("Mesure x_z prise : {}".format(x_z))

    print("Placez le y du bateau vers le haut, puis appuyez sur Entrée.")
    input()
    x_y = average_measurements_accel(imu)
    print("Mesure x_y prise : {}".format(x_y))

    print("Placez le z du bateau vers le bas, puis appuyez sur Entrée.")
    input()
    x_bas = average_measurements_accel(imu)
    print("Mesure x_bas prise : {}".format(x_bas))

    # Calcul du biais b (vecteur 3x1)
    b = -0.5 * (x_z + x_bas)
    print("Biais calculé : b = {}".format(b))

    # Calcul de B
    B = 9.81  # m.s**-2

    # Construction des vecteurs corrigés pour X
    x_x_corr = x_x + b
    x_y_corr = x_y + b
    x_z_corr = x_z + b

    # Construction de la matrice X 3x3 avec x_x_corr, x_y_corr, x_z_corr comme colonnes
    X = np.column_stack([x_x_corr, x_y_corr, x_z_corr])

    # Calcul de la matrice A (produit matriciel entre X et Y_inv)
    A = 1 / B * X
    print("Matrice A calculée : {}".format(A))

    return b, A

# Appel des fonctions de calibration
b_mag, A_mag = calib_mag()
b_accel, A_accel = calib_accel()

np.save("b_mag.npy", b_mag)
np.save("A_mag.npy", A_mag)
np.save("b_accel.npy", b_accel)
np.save("A_accel.npy", A_accel)



