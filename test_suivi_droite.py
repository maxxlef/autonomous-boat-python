import time
import sys
import os
import numpy as np
import csv
import quoicouroblib as rb

# Assure-toi que les modules sont correctement importés
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

import imu9_driver_v2 as imudrv
import arduino_driver_v2 as arddrv

# Création des objets IMU et Arduino
imu = imudrv.Imu9IO()
ard = arddrv.ArduinoIO()

###!!! A changer !!!###
M = [48.198724, -3.014025] 
A = [48.199170, -3.014700]

m = rb.projection(M[0],M[1])
a = rb.projection(A[0], A[1])
n = (a - m) / np.linalg.norm(a - m) # de M vers A

# Temps après avoir passé la bouée
temps_bouee = None
temps_suivi = 120  # 2 minutes = 120 secondes

while True:
    # Lecture des coordonnées GPS actuelles
    lat_p, long_p = rb.mesure_gps()
    p = rb.projection(lat_p,long_p)

    # Calcul de l'erreur et du cap
    cap_d = rb.cap_waypoint_2(m, n, p)

    # Distance à la bouée
    distance = rb.distance_droite(a, n, p)
    print("Distance de la bouée: {}".format(distance))
    if distance < 0:
        if temps_bouee is None:
            temps_bouee = time.time
        if time.time() - temps_bouee > temps_suivi:
            ard.send_arduino_cmd_motor(0,0)
            print("La bouée est passée, fin du suivi.")
            break
    # Calcul de la correction de cap et ajustement de la vitesse
    acc = rb.accel()
    bouss = rb.mag()
    rb.maintien_cap(acc, bouss, cap_d, 150)  # Suivre la droite avec une vitesse de base de 150
    # Pause pour éviter une boucle trop rapide
    time.sleep(0.2)