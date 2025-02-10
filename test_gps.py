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

rb.depart()
print("Debut")

try:
    while True:
        gll_ok,gll_data=gps.read_gll_non_blocking()
        if gll_ok:
            
            ### Données brutes ### -> DD*100 ou DM ????
            print("Donnees brutes :")
            print(gll_data)

            ### Données converties en degrés décimaux ###
            lattitude = rb.dm_to_dd(gll_data[0], gll_data[1])
            longitude = rb.dm_to_dd(gll_data[2], gll_data[3])
            print("Donnees en degrés décimaux :")
            print("Lattitude : {}".format(lattitude))
            print("Longitude : {}".format(longitude))

            ### Données converties en degrés minutes secondes ###
            lattitude = rb.dd_to_dms(gll_data[0], 'N')
            longitude = rb.dd_to_dms(gll_data[2], 'W')
            print("Donnees en degrés minutes secondes :")
            print("Lattitude : {}".format(lattitude))
            print("Longitude : {}".format(longitude))

        #### Récupérer les données brutes et les écrire dans un fichier .txt ###
        lat, dir_lat, long, dir_long = rb.mesure_gps()
        print("Donnees brutes enregistrees:")
        print("Lattitude : {}, Direction: {} ".format(lattitude, dir_lat))
        print("Longitude : {}, Direction {}".format(longitude, dir_long))

        time.sleep(1)

except KeyboardInterrupt:
    print("Programme interrompu par l'utilisateur.")
    print("Création du fichier CSV et du fichier geojson...")
    rb.create_csv("/mesures/gps_data.txt", "/mesures/gps_data.csv")
    rb.afficher_data("/mesures/gps_data.csv", "/mesures/gps_data.geojson")

    
