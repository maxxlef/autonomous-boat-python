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

print("Debut")

def mission_jour_2():
    lat_m=48.20010  
    long_m=-3.01573
    rb.suivre_vecteur(lat_m,long_m)
    
#mission_jour_2()
rb.reach_point(48.199014999999996, -3.0147983333333332)