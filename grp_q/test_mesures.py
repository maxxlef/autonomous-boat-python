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


print("Debut")
try:
    g1_hat = np.array([0,0,0])
    previous_time = time.time()
    while True:
        mag = rb.mag()
        print("Boussole : {}".format(mag))
        accel = rb.accel()
        print("Acceleration : {}".format(accel))
        euler = rb.angles_euler(accel, mag)
        phi = np.degrees(euler[0])
        theta = np.degrees(euler[1])
        psi = np.degrees(euler[2])
        print("Roulis1 : {} Tangage1 : {} Cap1 : {}".format(phi,theta,psi))
        gyro = rb.gyro()
        print("Gyroscope : {}".format(gyro))
        euler = rb.angles_euler_2(accel, mag,gyro,g1_hat)
        g1_hat = euler[3]
        print("Roulis2 : {} Tangage2 : {} Cap2 : {}".format(np.degrees(euler[0]),np.degrees(euler[1]),np.degrees(euler[2])))
        time.sleep(0.01)
except KeyboardInterrupt:
    print("Programme interrompu par l'utilisateur.")
    
