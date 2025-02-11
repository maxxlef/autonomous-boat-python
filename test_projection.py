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

"""
Ce fichier permet de rejoindre un point GPS donné en l'occurence par le point A.
Le fichier csv qui est retourné peut être tracé grâce au ficher 'tracer.py'

Fonctions utilisées dans quoicouroblib.py:

    def projection(ly,lx, lym = 48.199170, lxm = -3.014700):

    def mesure_gps(fichier="/mesures/gps_data.txt"):

    def accel():

    def mag():

"""

lat_a, long_a = 48.19891500000001, -3.013958333333333
a = rb.projection(lat_a, long_a)

print("Le point GPS voulu est : lattitude = {}, longitude = {}".format(lat_a, long_a))
print("Ces coordonnées dans le plan sont : x = {}, y = {}".format(a[0], a[1]))

filename = 'points_projection.csv'

# Ouvrir le fichier CSV en mode écriture
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['X', 'Y']) # Écrire l'en-tête du fichier
    
    try:
        while True:
            time.sleep(0.1)
            lat, long = rb.gps_dd()
            print("Mesure GPS du point p: lx ={}, ly ={}".format(lat, long))
            p = rb.projection(lat, long)
            writer.writerow([p[0], p[1]]) # Écrire les coordonnées
            print("Les coordonnées de P dans le plan : {}".format(p))
            print("Les coordonnées de A dans le plan : {}".format(a))
            d = a - p # Vecteur de P vers A
            distance = np.linalg.norm(d)
            print("Distance au point A : {}".format(distance))

            # Condition d'arrêt
            if rb.arret_waypoint(a, p, distance_min=1) == True:
                print("La bouee à atteint le point gps")


    except KeyboardInterrupt:
        print("Programme interrompu par l'utilisateur.")
