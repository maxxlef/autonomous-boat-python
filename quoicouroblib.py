import time
import sys
import os
import time
import numpy as np
import datetime
import csv
import json

# Assure-toi que les modules sont correctement importés
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

import imu9_driver_v2 as imudrv
import arduino_driver_v2 as arddrv
import gps_driver_v2 as gpsdrv

# Création des objets IMU et Arduino et GPS
imu = imudrv.Imu9IO()
ard = arddrv.ArduinoIO()
gps=gpsdrv.GpsIO()
gps.set_filter_speed("0")


def depart():
    """
    Attend qu'une accélération supérieure à 8 m/s² soit détectée sur l'axe x pour déclencher un départ.

    Input: None
    Output: None
    """
    t0 = time.time()
    while True:
        xaccel = accel()
        print(abs(time.time() - t0))
        if abs(xaccel[0]) > 8:
            break
        time.sleep(0.1)


def attendre_exact_heure(heure, minute):
    """
    Attend jusqu'à atteindre une heure précise (heure et minute) et affiche un compte à rebours.

    Input: heure (int), minute (int)
    Output: bool (True lorsque l'heure est atteinte)
    """
    maintenant = datetime.datetime.now()
    while True:      
        maintenant = datetime.datetime.now()
        heure_actuelle = maintenant.hour
        minute_actuelle = maintenant.minute
        delta = ((heure - heure_actuelle) * 3600) + ((minute - minute_actuelle) * 60) - maintenant.second
        if delta > 0:
            print("Attente de {} secondes pour atteindre {}h{}.".format(delta, heure, minute))
            time.sleep(1)
        else:
            break
    print("Il est {}h{} !".format(heure, minute))
    return True


def sawtooth(x):
    """
    Permet de limiter la valeur de x entre -pi et pi

    Input: x (float)
    Output: x (float)
    """
    return (x+np.pi)%(2*np.pi)-np.pi

def mag():
    xmag, ymag, zmag = imu.read_mag_raw()
    x = np.array([xmag, ymag, zmag])
    b = np.load("b_mag.npy")
    A = np.load("A_mag.npy")
    A_inv = np.linalg.inv(A)
    y = np.dot(A_inv,(x+b))
    return y # [x, y, z]

def accel():
    xaccel, yaccel, zaccel = imu.read_accel_raw()
    x = np.array([xaccel, yaccel, zaccel])
    b = np.load("b_accel.npy")
    A = np.load("A_accel.npy")
    A_inv = np.linalg.inv(A)
    y = np.dot(A_inv,(x+b))
    return y # [x, y, z]

def angles_euler(acc, mag):

    norm_acc = np.linalg.norm(acc)
    if norm_acc == 0:
        raise ValueError("La norme de l'accélération est nulle. Vérifie les données.")

    I = np.radians(64)  # Inclinaison magnétique locale
    mag_h = np.array([
        mag[0] * np.cos(I) + mag[2] * np.sin(I),  # Composante horizontale corrigée
        mag[1]
    ])

    psi = np.arctan2(mag_h[1], mag_h[0])
    phi = np.arcsin(acc[1] / norm_acc)
    theta = -np.arcsin(acc[0] / norm_acc)

    return np.array([phi, theta, psi]) # [Tangage, Roulis, Cap]

def maintien_cap(acc,mag,cap,spd_base,debug=False):
    psi = angles_euler(acc,mag)[2]
    err = sawtooth(cap-psi)
    Kd = 100
    correction = Kd*err
    spd_left = spd_base + correction
    spd_right = spd_base - correction
    if spd_left < 0:
        spd_left = 0
    if spd_right < 0:
        spd_right = 0
    if spd_left > 255:
        spd_left = 255
    if spd_right > 255:
        spd_right = 255
    if debug:
        print("Cap actuel du bateau: {}°, erreur: {}°".format(np.degrees(psi), np.degrees(err)))
        print("Speed left = {}".format(spd_left))
        print("Speed right = {}".format(spd_right))
    ard.send_arduino_cmd_motor(spd_left,spd_right)

def dd_to_dms(dd, direction):
    minutes = (dd - int(dd)) * 60
    secondes = (minutes - int(minutes)) * 60
    if direction in ['S', 'W']:
        dd *= -1
    dms = '{}°{}\'{:.2f}"'.format(int(dd), int(minutes), secondes)
    return dms

def dm_to_dd(dm,direction):
    degrees = int(dm // 100)
    minutes = dm - degrees * 100
    dd = degrees + (minutes / 60)
    if direction in ['S', 'W']:
        dd *= -1
    return dd

def dms_to_dd(dms, direction): # format: dms = '{}°{}\'{:.2f}"'.format(int(dd), int(minutes), secondes)'
    dms = dms.replace('°', ' ').replace('\'', ' ').replace('"', ' ')
    d, m, s = map(float, dms.split())
    dd = d + m / 60 + s / 3600
    if direction in ['S', 'W']:
        dd *= -1
    return dd

def mesure_gps(fichier="/mesures/gps_data.txt"):
    with open(fichier, "a") as file:  # Ouvrir le fichier en mode ajout (append)
        while True:
            gll_ok, gll_data = gps.read_gll_non_blocking()
            if gll_ok:
                file.write("{}\n".format(gll_data)) # Écrire les données brutes
                return gll_data
    
def create_csv(input_file, output_csv_path):

    with open(input_file, 'r') as file:
        file_content = file.read()
    with open(output_csv_path, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['Latitude', 'Longitude'])
        
        # Lire chaque ligne du fichier .txt
        for line in file_content.strip().split('\n'):
            data = eval(line.strip())
            
            # Extraire la latitude et la longitude au format degré-minutes (ou dd*100 ?)
            latitude_dm, lat_dir, longitude_dm, long_dir = data[:4]
            
            latitude = dm_to_dd(latitude_dm, lat_dir)
            longitude = dm_to_dd(longitude_dm, long_dir)

            csv_writer.writerow([latitude, longitude])

    print("Fichier CSV généré avec succès : {}".format(output_csv_path))

def afficher_data(csv_file, output_geojson_file):
    # Initialiser une structure GeoJSON
    geojson_data = {
        "type": "FeatureCollection",
        "features": []
    }

    # Ouvrir le fichier CSV et lire les coordonnées GPS
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            latitude = float(row['Latitude'])
            longitude = float(row['Longitude'])
            
            # Créer une nouvelle Feature (point) GeoJSON pour chaque ligne du CSV
            feature = {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [longitude, latitude]  # Les coordonnées GeoJSON sont [long, lat] en DD
                },
                "properties": {}  # Ajouter des propriétés supplémentaires ici si nécessaire
            }
            # Ajouter la feature à la collection
            geojson_data["features"].append(feature)

    # Écrire le GeoJSON dans un fichier
    with open(output_geojson_file, 'w') as geojson_file:
        json.dump(geojson_data, geojson_file, indent=4)

    print("Fichier GeoJSON généré avec succès : {}".format(output_geojson_file))

def projection(lat,long, lat_m = 48.199170, long_m = -3.014700):# Format degrés decimaux
    
    """
    Convertit les coordonnées GPS (latitude, longitude en format degrés décimaux) 
    en coordonnées cartésiennes locales par rapport à un point M défini par
    lat_m et long_m (en degrés décimaux), en ne retournant que x et y.
    """

    rho = 6371009.7714

    # Conversion des latitudes et longitudes en radians
    lat_m_rad = np.radians(lat_m)
    long_m_rad = np.radians(long_m)
    lat_rad = np.radians(lat)
    long_rad = np.radians(long)
    # Conversion des coordonnées du point M (centre) en cartésiennes 2D (x_m, y_m)
    x_m = rho * np.cos(lat_m_rad) * np.cos(long_m_rad)
    y_m = rho * np.cos(lat_m_rad) * np.sin(long_m_rad)
    # Conversion des coordonnées du point P en cartésiennes 2D (x_p, y_p)
    x_p = rho * np.cos(lat_rad) * np.cos(long_rad)
    y_p = rho * np.cos(lat_rad) * np.sin(long_rad)
    # Calcul des coordonnées relatives par rapport au point M
    x = x_p - x_m
    y = y_p - y_m
    p = np.array([x,y])
    return p

def cap_waypoint(a,p):
    """
    Calcule le cap à suivre pour atteindre un waypoint A depuis un point P.
    """
    d = a - p
    n = d / np.linalg.norm(d)
    cap_d = -np.arctan2(n[1],n[0]) # Sans le " - " le bateau tourne dans le sens trigonométrique
    return cap_d

def arret_waypoint(a,p, distance_min = 2):
    distance = np.linalg.norm(a - p)
    if distance < distance_min:
        return True
    else:
        return False
    
def regulation_vitesse(distance, vmax=200, vmin=45, coef=1, middle=4):
    """
    Régulation de la vitesse en fonction de la distance
    Peut être ajustée avec test_regulation_vitesse.py
    """
    vitesse = (vmax-vmin) * (0.5 * (1 - np.tanh(coef * (middle - distance)))) + vmin
    return max(0, min(vmax, vitesse))

