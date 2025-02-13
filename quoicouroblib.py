import time
import sys
import os
import time
import numpy as np
import datetime
import csv
import json
import socket

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
    """
    Lit les données brutes du magnétomètre et les corrige avec les matrices de calibration.
    
    Input: None
    
    Output: y (np.array)
    """
    xmag, ymag, zmag = imu.read_mag_raw()
    x = np.array([xmag, ymag, zmag])
    b = np.load("b_mag.npy")
    A = np.load("A_mag.npy")
    A_inv = np.linalg.inv(A)
    y = np.dot(A_inv,(x+b))
    return y # [x, y, z]

def accel():
    """
    Lit les données brutes de l'accéléromètre et les corrige avec les matrices de calibration.

    Input: None

    Output: y (np.array)
    """
    xaccel, yaccel, zaccel = imu.read_accel_raw()
    x = np.array([xaccel, yaccel, zaccel])
    b = np.load("b_accel.npy")
    A = np.load("A_accel.npy")
    A_inv = np.linalg.inv(A)
    y = np.dot(A_inv,(x+b))
    return y # [x, y, z]

def gyro(filtre):
    xgyro, ygyro, zgyro = imu.read_gyro_raw()
    return np.array([xgyro, ygyro, zgyro])/938.0

def angles_euler(acc, mag):
    """
    Calcule les angles d'Euler (tangage, roulis, cap) à partir des données d'accélération et de magnétomètre.
    
    Input: acc (np.array), mag (np.array)
    
    Output: euler (np.array)
    """
    phi = np.arcsin(acc[1]/np.linalg.norm(acc))  # Angle de roulis
    theta = -np.arcsin(acc[0]/np.linalg.norm(acc))  # Angle de tangage
    psi = np.arctan2(mag[1],mag[0])
    return [phi, theta, psi]

def rotuv(u,v): #returns rotation with minimal angle  such that  v=R*u
            # see https://en.wikipedia.org/wiki/Rotation_matrix#Vector_to_vector_formulation
    u=np.array(u).reshape(3,1)
    v=np.array(v).reshape(3,1)
    u=(1/np.linalg.norm(u))*u
    v=(1/np.linalg.norm(v))*v
    c=scalarprod(u,v)
    A=v@u.T-u@v.T
    return np.eye(3,3)+A+(1/(1+c))*A@A

def scalarprod(u,v): # scalar product
    u,v=u.flatten(),v.flatten()
    return sum(u[:]*v[:])

def adjoint(w):
    """
    Retourne la matrice adjointe de w.

    Input: w (np.array) ou (float)

    Output: np.array(3,3) ou np.array(2,2) si float
    """
    if isinstance(w, (float, int)): return np.array([[0,-w] , [w,0]])
    #print('tolist(w)=',w)
    return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

def angles_euler_2(a1, y1, w1, g1_hat):
    """
    Calcule les angles d'Euler (tangage, roulis, cap) à partir des données
    d'accélération, de magnétomètre et de gyroscope.

    - a1 : vecteur d'accélération mesuré (3,)
    - y1 : vecteur du magnétomètre mesuré (3,)
    - w1 : vecteur de vitesse angulaire (gyroscope) (3,)
    - g1_hat : estimation précédente de la direction de la gravité (3,)
    Retourne : (ϕ, θ, ψ) correspondant aux angles d'Euler (roulis, tangage, cap)
    """
    g0 = np.array([0, 0, 1])  # Direction de la gravité dans le repère de référence
    y1_n = y1 / np.linalg.norm(y1)  # Normalisation du vecteur magnétomètre
    λ = 0.99  # Coefficient de filtrage
    dt = 0.01

    # Mise à jour de l'estimation de la gravité
    g1_hat = λ * (np.eye(3) - dt * adjoint(w1)) @ g1_hat + (1 - λ) * a1
    g1_n = g1_hat / np.linalg.norm(g1_hat)  # Normalisation de g1_hat

    # Calcul des angles d'Euler
    ϕ = np.arcsin(np.dot([0, 1, 0], g1_n))  # Roulis
    θ = -np.arcsin(np.dot([1, 0, 0], g1_n))  # Tangage

    # Rotation du vecteur magnétomètre pour alignement avec le repère de référence
    Rh = rotuv(g1_n, g0)
    yh = Rh @ y1_n

    # Calcul du cap (ψ) en utilisant l'arctangente
    ψ = np.arctan2(yh[1], yh[0])

    return [ϕ, θ, ψ, g1_hat]

def dd_to_dms(dd, direction):
    """
    Convertit des coordonnées au format degrés décimaux en degrés minutes secondes.

    Input: dd (float), direction (str)

    Output: dms (float)
    """
    minutes = (dd - int(dd)) * 60
    secondes = (minutes - int(minutes)) * 60
    if direction in ['S', 'W']:
        dd *= -1
    dms = '{} {}\'{:.2f}"'.format(int(dd), int(minutes), secondes)
    return dms

def dm_to_dd(dm,direction):
    """
    Convertit des coordonnées au format degrés minutes en degrés décimaux.

    Input: dm (float), direction (str)

    Output: dd (float)
    """

    degrees = int(dm // 100)
    minutes = dm - degrees * 100
    dd = degrees + (minutes / 60)
    if direction in ['S', 'W']:
        dd *= -1
    return dd

def dms_to_dd(dms, direction): # format: dms = '{}°{}\'{:.2f}"'.format(int(dd), int(minutes), secondes)'
    """
    Convertit des coordonnées au format degrés minutes secondes en degrés décimaux.

    Input: dms (str) (format: '{}°{}\'{:.2f}"'.format(int(dd), int(minutes), secondes)), direction (str)

    Output: dd (float)
    """
    dms = dms.replace('°', ' ').replace('\'', ' ').replace('"', ' ')
    d, m, s = map(float, dms.split())
    dd = d + m / 60 + s / 3600
    if direction in ['S', 'W']:
        dd *= -1
    return dd

def mesure_gps(fichier="gps_data.txt"):
    """
    Lit les données GPS brutes et les écrit dans un fichier .txt.

    Input: 
        - fichier (str) : Chemin du fichier où les données seront écrites.
    
    Output: 
        - gll_data (tuple) : Données GPS lues.
    """
    # Ouvrir le fichier en mode ajout (append) et écrire les données
    with open(fichier, "a") as file:
        while True:
            gll_ok, gll_data = gps.read_gll_non_blocking()
            if gll_ok:
                file.write("{}\n".format(gll_data))  # Écrire les données brutes
                print(gll_data)
                return gll_data[0],gll_data[1],gll_data[2],gll_data[3]
            
def gps_dd():
    lat,dir_lat,long,dir_long = mesure_gps()
    lat= dm_to_dd(lat,dir_lat)
    long = dm_to_dd(long,dir_long)
    return lat,long
   
def create_csv(input_file, output_csv_path):
    """
    Convertir un fichier .txt contenant des données GPS brutes en un fichier CSV

    Input: input_file (str), output_csv_path (str)
    
    Output: None (enregistre un fichier CSV)
    """
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

            csv_writer.writerow([latitude, longitude]) # Écrire les données en format degrés décimaux

def afficher_data(csv_file, output_geojson_file):
    """
    Convertit un fichier CSV contenant des coordonnées GPS en un fichier GeoJSON.
    Le fichier CSV doit contenir les colonnes 'Latitude' et 'Longitude',
    en format degrés décimaux.

    Input: csv_file (str), output_geojson_file (str)

    Output: None (enregistre un fichier GeoJSON)
    """
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

def projection(lat,long, lat_m = 48.199014999999996, long_m = -3.0147983333333336):# Format degrés decimaux
    """
    Convertit les coordonnées GPS (latitude, longitude en format degrés décimaux) 
    en coordonnées cartésiennes locales par rapport à un point M défini par
    lat_m et long_m (en degrés décimaux), en ne retournant que x et y.

    Input: lat (float), long (float), lat_m (float), long_m (float)

    Output: p (np.array)
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
    cap_d = np.pi - np.arctan2(n[1],n[0]) # À vérifier si c'est bien pi/2 - arctan2(n[1],n[0])
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

    Input: distance (float), vmax (float), vmin (float), coef (float), middle (float)

    Output: vitesse (float)
    """
    vitesse = (vmax-vmin) * (0.5 * (1 - np.tanh(coef * (middle - distance)))) + vmin
    return max(0, min(vmax, vitesse))

def reach_point(lat_a, long_a, debug=True):
    """
    Rejoindre un point GPS donné par le point A en format degrés décimaux.

    Input: lat_a (float), long_a (float)

    Output: None
    """
    a = projection(lat_a, long_a)
    while True:
        time.sleep(0.5)
        lat, long = gps_dd()
        p = projection(lat, long)
        d = a - p # Vecteur de P vers A

        # Correction du cap
        cap_d = cap_waypoint(a, p)
        distance = np.linalg.norm(d)
        acc = accel()
        bouss = mag()
        spd = regulation_vitesse(distance)
        maintien_cap(acc, bouss, cap_d, spd)

        # Condition d'arrêt
        if arret_waypoint(a, p) == True:
            print("La bouee a atteint le point gps")
            ard.send_arduino_cmd_motor(0, 0)
            break

        if debug:
            print("-----------------------------")
            print("Le point GPS voulu est : lattitude = {}, longitude = {}".format(lat_a, long_a))
            print("Ces coordonnees dans le plan sont : x = {}, y = {}".format(a[0], a[1]))
            print("---")
            print("Mesure GPS du point p: lat ={}, long ={}".format(lat, long))
            print("Les coordonnees de P dans le plan : {}".format(p))
            print("---")
            print("Distance au point A : {}".format(distance))
            print("Cap vise par cap_d : {}".format(np.degrees(cap_d)))
            print("Cap vise par cap_d : {}".format(np.degrees(cap_d)))

def cap_waypoint_2(m,n,p, debug=False):
    """
    Calcule le cap pour suivre une droite définie par
    un point A et un vecteur directeur n, depuis un point P.

    Input: a (np.array), p (np.array), n (np.array)

    Output: cap_d (float)
    """
    e = n[0]*(p[1]-m[1]) - n[1]*(p[0]-m[0]) 
    phi = np.arctan2(n[1],n[0]) 
    phi = np.pi/2 - phi
    # phi était l'angle entre l'axe x et le vecteur n (arctan2(n[1],n[0]))
    # mais x est dirigé vers l'Est et non vers le Nord
    # donc on doit le tourner de 90° dans le sens trigonométrique
    # pour avoir le bon cap (avec 0° vers le Nord)
    k = 2
    cap_d = phi - k*np.tanh(e/4) # k est un coefficient à ajuster et vérifier le signe - ou +
    if debug:
        print("Erreur: {}".format(e))
        print("Phi: {}".format(np.degrees(phi)))
        print("Cap a suivre: {}".format(np.degrees(cap_d)))
    return cap_d

def suivre_droite(M, A, debug=True): #  M (départ) et A (fin) en format degrés décimaux
    """
    Permet de suivre une droite définie par un point M et un point A
    en format degrés décimaux.

    Input: M (list), A (list)

    Output: None
    """
    m = projection(M[0],M[1])
    a = projection(A[0], A[1])
    n = (a - m) / np.linalg.norm(a - m) # de M vers A

    while True:
        lat_p, long_p = mesure_gps()
        p = projection(lat_p,long_p)

        # Calcul du cap désiré
        cap_d = cap_waypoint_2(m, n, p)

        distance = distance_droite(a, n, p)
        if distance < 0:
            ard.send_arduino_cmd_motor(0, 0)
            print("La bouee est passee, fin du suivi.")
            return

        acc = accel()
        bouss = mag()
        maintien_cap(acc, bouss, cap_d, 150)
        time.sleep(0.2)

        if debug:
            print("-----------------------------")
            print("Les coordonnees de P dans le plan : {}".format(p))
            print("Les coordonnees de A dans le plan : {}".format(a))
            print("---")
            print("Distance au point A : {}".format(distance))
            print("Cap vise par cap_d : {}".format(np.degrees(cap_d)))
    
def distance_droite(a, n, p):
    """
    Calcule la distance entre un point P et une droite définie
    par un point A et un vecteur normal n. La distance est positive
    si le point P est à gauche de la droite (sens du vecteur n),

    Input: a (np.array), n (np.array), p (np.array)

    Output: distance (float)
    """
    vecteur_pa = a - p
    distance = np.cross(n, vecteur_pa) / np.linalg.norm(n)
    return distance

def maintien_cap(acc,mag,cap,spd_base,debug=True):
    """
    Maintient le cap du bateau en ajustant la vitesse des moteurs en fonction de l'erreur entre le cap actuel et le cap voulu.

    Input: acc (np.array), mag (np.array), cap (float), spd_base (int), debug (bool)

    Output: None

    On a mis le kd à 50 lors du suivi de point extremement lent.
    Initiallement kd=100 pour des vitesses plus vite
    """
    psi = angles_euler(acc,mag)[2]
    err = sawtooth(cap-psi)
    Kd = 50
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
        print("-----------------------------")
        print("Cap actuel du bateau: {}".format(np.degrees(psi)))
        print("Cap voulu: {}".format(np.degrees(cap)))
        print("Erreur_cap: {}".format(np.degrees(err)))
        print("---")
        print("Speed de base: {}".format(spd_base))
        print("Speed left = {}".format(spd_left))
        print("Speed right = {}".format(spd_right))
    ard.send_arduino_cmd_motor(spd_left,spd_right)

def maintien_cap_2(rb,cap,spd_base,debug=True):
    """
    Maintient le cap du bateau en ajustant la vitesse des moteurs en fonction de l'erreur entre le cap actuel et le cap voulu.

    Input: acc (np.array), mag (np.array), cap (float), spd_base (int), debug (bool)

    Output: None
    """
    g1_hat = np.array([0,0,0])
    mag = rb.mag()
    #print("Boussole : {}".format(mag))
    accel = rb.accel()
    #print("Acceleration : {}".format(accel))
    sensor_filter = SensorFilter(window_size=10)
    gyro = rb.gyro(sensor_filter)
    #print("Gyroscope : {}".format(gyro))
    euler = rb.angles_euler_2(accel, mag,gyro,g1_hat)
    psi=euler[2]
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
        print("-----------------------------")
        print("Cap actuel du bateau: {}".format(np.degrees(psi)))
        print("Cap voulu: {}".format(np.degrees(cap)))
        print("Erreur_cap: {}".format(np.degrees(err)))
        print("---")
        print("Speed de base: {}".format(spd_base))
        print("Speed left = {}".format(spd_left))
        print("Speed right = {}".format(spd_right))
    ard.send_arduino_cmd_motor(spd_left,spd_right)
    time.sleep(0.1)

def cercle(n,t,lat_boue,long_boue,k1=20,k2 = 20, r=30, T=330,debug=True):
    lat,long = gps_dd()
    lx,ly=projection(lat,long)
    N=20

    lat_boue,long_boue = projection(lat_boue,long_boue)
    m = np.array([[lat_boue],[long_boue]])

    p = np.array([[lx],[ly]])
    p_tilde= m +r * np.array([[np.cos(2*np.pi*((t)/T + n/N))],
                             [np.sin(2*np.pi*((t)/T +n/N))]])
    e = p_tilde - p
    
    v_tilde= r * (2*np.pi/T) * np.array([[-np.sin(2*np.pi*((t)/T +n/N))],
                                         [np.cos(2*np.pi*((t)/T +n/N))]])
    
    speed = k1 * np.linalg.norm(v_tilde) + regulation_vitesse(np.linalg.norm(p_tilde-p), vmax=170, vmin=35, coef=0.3, middle=10)
    
    d =   e + k2 * v_tilde
    d =   e + k2 * v_tilde
    phi = np.arctan2(d[1,0],d[0,0])
    cap_d = np.pi - phi

    if debug:
        print("norme v_tilde : {}".format(np.linalg.norm(v_tilde)))
        print("norme p_tilde moins p : {}".format(np.linalg.norm(e)))
        print("cap_d : {}".format(np.degrees(cap_d)))
        print("norme v_tilde : {}".format(np.linalg.norm(v_tilde)))
        print("norme p_tilde moins p : {}".format(np.linalg.norm(e)))
        print("cap_d : {}".format(np.degrees(cap_d)))
        print("speed bateau: {}".format(speed))
    return speed, cap_d , p_tilde , p
    return speed, cap_d , p_tilde , p

def suivre_vecteur(n,t0,lat_m,long_m,boucle = True):
    while True:
        t=time.time()-t0
        speed, cap_d, p_tilde,p= cercle(n,t,lat_m,long_m)
        speed, cap_d, p_tilde,p= cercle(n,t,lat_m,long_m)
        bouss = mag()
        acc = accel()   
        maintien_cap(acc,bouss,cap_d,speed)
        time.sleep(0.1)
        if not boucle:
            return p_tilde ,p

            return p_tilde ,p

def robot2_client_onetime(server_ip):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, 5000))
    print("Connected to DDBoat 17 (Server)")

    # def signal_handler(sig, frame):
    #     print("Closing client connection gracefully...")
    #     client_socket.close()
    #     sys.exit(0)
    
    # signal.signal(signal.SIGINT, signal_handler)

    data = ""
    try:
        while True:
            data = client_socket.recv(1024).decode()
            if not data:
                break
            print("Received GPS Position from DDBoat 17: {}".format(data))
            break
    except Exception as e:
        print("Client error: {}".format(e))
    finally:
        client_socket.close()
    try:
        list_data = data.split(";")
        lat , dir_lat, long, dir_long = float(list_data[0]), 'N', float(list_data[2]), 'W'
        lat = dm_to_dd(lat,dir_lat)
        long = dm_to_dd(long,dir_long)
        print("Lattiude = {}, Longitude = {}".format(lat, long))
        print("lat = {}, dir_lat = {}, long = {}, dir_long = {}".format(lat, dir_lat, long, dir_long))
        print("Lattiude = {}, Longitude = {}".format(lat, long))
        return lat, long
    except:
        pass

def calc_b_A_from_file(filename):
    """
    Calcule b et A à partir des données d'un fichier .txt contenant x_nord, x_sud, x_west, x_up.
    
    Args:
        filename (str): Nom du fichier .txt à lire.

    Returns:
        tuple: b (vecteur 3x1), A (matrice 3x3)
    """
    # Lecture des données dans le fichier
    data = np.loadtxt(filename).reshape(-1, 1)  # data est une matrice 12x1

    # Reconstruction des vecteurs 3D
    x_nord = data[0:3].flatten()  # Première tranche : 0, 1, 2
    x_sud = data[3:6].flatten()   # Deuxième tranche : 3, 4, 5
    x_west = data[6:9].flatten()  # Troisième tranche : 6, 7, 8
    x_up = data[9:12].flatten()   # Quatrième tranche : 9, 10, 11

     # Calcul du biais b (vecteur 3x1)
    b = -0.5 * (x_nord + x_sud)
    print("Biais calcule : b = {}".format(b))

    # Calcul de B et I
    B = 46 * 10**(-6)  # T
    I = 64 * np.pi / 180  # rad

    # Calcul des y (vecteurs théoriques, chaque vecteur est 3x1)
    y_nord = np.array([B * np.cos(I), 0, -B * np.sin(I)])
    y_west = np.array([0, -B * np.cos(I), -B * np.sin(I)])
    y_up = np.array([-B * np.sin(I), 0, -B * np.cos(I)])

    # Construction de la matrice Y 3x3 avec y_nord, y_west, y_up comme colonnes
    Y = np.column_stack([y_nord, y_west, y_up])

    # Calcul de l'inverse de la matrice Y
    Y_inv = np.linalg.inv(Y)

    # Construction des vecteurs corrigés pour X
    x_nord_corr = x_nord + b
    x_west_corr = x_west + b
    x_up_corr = x_up + b

    # Construction de la matrice X 3x3 avec x_nord_corr, x_west_corr, x_up_corr comme colonnes
    X = np.column_stack([x_nord_corr, x_west_corr, x_up_corr])

    # Calcul de la matrice A (produit matriciel entre X et Y_inv)
    A = np.dot(X, Y_inv)
    print("Matrice A calculee : {}".format(A))
    np.save("A_mag.npy", A)
    np.save("b_mag.npy", b)
