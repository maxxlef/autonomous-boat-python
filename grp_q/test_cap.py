import time
import sys
import os
import time
import numpy as np
import quoicouroblib as rb

# Assure-toi que les modules sont correctement importés
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

import imu9_driver_v2 as imudrv
import arduino_driver_v2 as arddrv

# Création des objets IMU et Arduino
imu = imudrv.Imu9IO()
ard = arddrv.ArduinoIO()

"""
Fichier pour tester la calibration de notre boussole

Fonctions utilisées dans quoicouroblib.py:

    def depart():
    
    def accel():
    
    def mag():
    
    def maintien_cap(acc, bouss, cap_voulu, spd, debug=False):
        
        def angles_euler_2(a1, y1, w1, g1_hat):
        ou
        def angles_euler(acc, mag):
    
"""

cap_desire_deg = float(input("Rentrer le cap desire en degres :"))
cap_desire = np.radians(cap_desire_deg)

rb.depart()
print("Debut")

try:
    while True:
        print("Le cap voulu est : {}".format(cap_desire_deg))
        acc = rb.accel()
        bouss = rb.mag()
        spd = 0
        rb.maintien_cap(acc,bouss,cap_desire,spd,True)
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Programme interrompu par lutilisateur.")








