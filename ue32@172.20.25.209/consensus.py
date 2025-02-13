import socket
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
numero_bateau = socket.gethostname()[6:8]
filename = "calib_" + numero_bateau + ".txt"
print(filename)
rb.calc_b_A_from_file(filename)
rb.reach_point(48.20010,-3.01573 ) #bouee
rb.reach_point(48.199014999999996, -3.0147983333333332) #ponton
