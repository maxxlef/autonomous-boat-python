import sys
import os
import time

# Assure-toi que les modules sont correctement importés
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

import imu9_driver_v2 as imudrv
import arduino_driver_v2 as arddrv

# Création des objets IMU et Arduino
imu = imudrv.Imu9IO()
ard = arddrv.ArduinoIO()

### Arrêt du bateau ###
spdleft = 0
spdright = 0
ard.send_arduino_cmd_motor(spdleft, spdright)