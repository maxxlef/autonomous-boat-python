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

def mission_jour_3():
    """
    Tourne autour du bateau 
    """
    #rb.attendre_exact_heure(11,30)
    while True:
        numero_bateau = "17"
        ip_adresse = "172.20.25.2" + numero_bateau
        lat_m, long_m = rb.robot2_client_onetime(ip_adresse) #En degrés décimaux
        rb.suivre_vecteur(lat_m,long_m,boucle = False)
    
def mission2_jour_3():
    """
    Rejoint un bateau
    """
    debug = True


    while True:
        numero_bateau = "17"
        ip_adresse = "172.20.25.2" + numero_bateau
        lat_m, long_m = rb.robot2_client_onetime(ip_adresse) #En degrés décimaux
        m = rb.projection(lat_m, long_m)

        lat, long = rb.gps_dd()
        p = rb.projection(lat, long)
        d = m - p # Vecteur de P vers A

        # Correction du cap
        cap_d = rb.cap_waypoint(m, p)
        distance = np.linalg.norm(d)
        acc = rb.accel()
        bouss = rb.mag()
        vmax = 220
        coef = 0.6 # Particie à l'écrasement de la tangente hyperbolique (en le résuisant)
        middle = 12 # Distance à laquelle la vitesse est moyenne
        vmin = 45
        spd = rb.regulation_vitesse(distance, vmax, vmin, coef, middle)
        rb.maintien_cap(acc, bouss, cap_d, spd)

        if debug:
            print("-----------------------------")
            print("Le point GPS voulu est : lattitude = {}, longitude = {}".format(lat_m, long_m))
            print("Ces coordonnees dans le plan sont : x = {}, y = {}".format(m[0], m[1]))
            print("---")
            print("Mesure GPS du point p: lat ={}, long ={}".format(lat, long))
            print("Les coordonnees de P dans le plan : {}".format(p))
            print("---")
            print("Distance au point A : {}".format(distance))
            print("Cap vise par cap_d : {}".format(np.degrees(cap_d)))
            print("Cap vise par cap_d : {}".format(np.degrees(cap_d)))


mission_jour_3()
#mission2_jour_3()
#rb.reach_point(48.199014999999996, -3.0147983333333332)