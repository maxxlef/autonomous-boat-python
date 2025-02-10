import sys
import os
import time
import quoicouroblib as rb
import numpy as np

# Assure-toi que les modules sont correctement importés
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

import imu9_driver_v2 as imudrv
import arduino_driver_v2 as arddrv
import gps_driver_v2 as gpsdrv

# Création des objets IMU et Arduino
imu = imudrv.Imu9IO()
ard = arddrv.ArduinoIO()

"""
Fichier pour comprendre quelles sont les données renvoyées par les cateurs

Fonction utilisée dans quoicouroblib.py:

    def mag():

    def accel():

    def angles_euler(acc, mag):

"""


print("Début")
try:
    while True:
        x_mag = rb.mag()
        print("Boussole : {}".format(x_mag))
        x_accel = rb.accel()
        print("Acceleration : {}".format(x_accel))
        euler = rb.angles_euler(x_accel, x_mag)
        phi = np.degrees(euler[0])
        theta = np.degrees(euler[1])
        psi = np.degrees(euler[2])
        print("Tangage : {} Roulis : {} Cap : {}".format(phi,theta,psi))
        time.sleep(1)
except KeyboardInterrupt:
    print("Programme interrompu par l'utilisateur.")
    
