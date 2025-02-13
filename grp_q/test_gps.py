import sys
import os
import time
import quoicouroblib as rb

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import gps_driver_v2 as gpsdrv

"""
Ce programme permet de tester les données GPS et les conversions en degrés décimaux.

Fonctions utilisées dans quoicouroblib.py:

    def dd_to_dms(dd, direction):

    def dms_to_decimal(dms, direction):

    def mesure_gps(fichier="/mesures/gps_data.txt"):
    
"""


gps=gpsdrv.GpsIO()
gps.set_filter_speed("0")

print("Debut")

try:
    while True:

        #### Récupérer les données brutes et les écrire dans un fichier .txt ###
        lattitude, dir_lat, longitude, dir_long = rb.mesure_gps()
        print("Donnees brutes enregistrees:")
        print("Lattitude : {}, Direction: {} ".format(lattitude, dir_lat))
        print("Longitude : {}, Direction {}".format(longitude, dir_long))

        lat = rb.dm_to_dd(lattitude, dir_lat)
        long = rb.dm_to_dd(longitude,dir_long)
        print("Donnees converties en degres decimaux:")
        print("Lattitude : {}, Direction: {}".format(lat, dir_lat))
        print("Longitude : {}, Direction : {}".format(long, dir_long))
        time.sleep(1)

except KeyboardInterrupt:
    print("Programme interrompu par l'utilisateur.")
    print("Création du fichier CSV et du fichier geojson...")
    rb.create_csv("/mesures/gps_data.txt", "/mesures/gps_data.csv")
    rb.afficher_data("/mesures/gps_data.csv", "/mesures/gps_data.geojson")

    
