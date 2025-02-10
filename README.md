
## Notes ##
Les données GPS sont automatiquement sauvegardées à la suite d'un fichier .txt avec `mesure_gps()`, il faut donc penser à le supprimer avant de vouloir faire une mesure propre.
![Coordonnées](Image_rapport/Notes1.png)
![Projection](Image_rapport/Notes2.png)
![Suivi_de_ligne](Image_rapport/Notes3.png)
![TO_DO](Image_rapport/Notes4.png)


## **quoicouroblib.py**
### `depart()`
Attend qu'une accélération supérieure à 8 m/s² soit détectée sur l'axe x pour déclencher un départ.
- **Input**: Aucun
- **Output**: Aucun

---

### `attendre_exact_heure(heure, minute)`
Attend jusqu'à atteindre une heure précise (heure et minute) et affiche un compte à rebours.
- **Input**:
  - `heure` (int): Heure souhaitée
  - `minute` (int): Minute souhaitée
- **Output**: 
  - `True` lorsque l'heure est atteinte

---

### `sawtooth(x)`
Permet de limiter la valeur de x entre -pi et pi.
- **Input**: 
  - `x` (float)
- **Output**: 
  - `x` (float)

---

### `mag()`
Récupère les données du magnétomètre et les corrige à l'aide de matrices de calibration.
- **Input**: Aucun
- **Output**: 
  - `y` (np.array) - Champ magnétique corrigé [x, y, z]

---

### `accel()`:
Calcule la valeur corrigée de l'accélération brute en utilisant les matrices de correction.
- **Input**: Aucun
- **Output**: `y` (np.array) - Accélération corrigée [x, y, z]

---

### `angles_euler(acc, mag)`:
Calcule les angles d'Euler (Tangage, Roulis, Cap) à partir des données d'accélération et de magnétomètre.
- **Input**:
  - `acc` (np.array): Vecteur d'accélération corrigé [x, y, z]
  - `mag` (np.array): Vecteur de champ magnétique corrigé [x, y, z]
- **Output**:
  - `angles` (np.array): Angles d'Euler [Tangage, Roulis, Cap]

---

### `rotuv(u, v)`:
Retourne la rotation avec l'angle minimal tel que \( v = R \cdot u \).
- **Input**: 
  - `u` : Vecteur 3D d'entrée
  - `v` : Vecteur 3D cible
- **Output**: Matrice de rotation \( R \in \mathbb{R}^{3 \times 3} \)

---

### `angles_euler_2(a1, y1, w1, g1_hat)`:
Calcule les angles d'Euler (tangage, roulis, cap) à partir des données d'accélération, de magnétomètre et de gyroscope.
- **Input**: 
  - `a1` : Vecteur accélération
  - `y1` : Vecteur magnétomètre
  - `w1` : Vecteur gyroscope
  - `g1_hat` : Estimation précédente de \( g_1 \)
- **Output**: Angles d'Euler \( \varphi \) (tangage), \( \theta \) (roulis), et \( \psi \) (cap)


### `maintien_cap(acc, mag, cap, spd_base, debug=False)`:
Maintient le cap en ajustant la vitesse des moteurs gauche et droite pour compenser l'erreur angulaire.
- **Input**:
  - `acc` (np.array): Vecteur d'accélération corrigé [x, y, z]
  - `mag` (np.array): Vecteur de champ magnétique corrigé [x, y, z]
  - `cap` (float): Cap désiré en radians
  - `spd_base` (int): Vitesse de base des moteurs
  - `debug` (bool): Affichage des informations de débogage (facultatif)
- **Output**: Aucun

---

### `dd_to_dms(dd, direction)`:
Convertit les coordonnées en degrés décimaux (DD) vers le format degrés, minutes, secondes (DMS).
- **Input**:
  - `dd` (float): Coordonnée en degrés décimaux
  - `direction` (str): Direction ('N', 'S', 'E', 'W') pour appliquer le signe correct
- **Output**: 
  - `dms` (str) - Coordonnée au format degrés, minutes, secondes (DMS)

---

### `dm_to_dd(dm, direction)`:
Convertit les coordonnées en degrés-minutes (DM) vers le format degrés décimaux (DD).
- **Input**:
  - `dm` (float): Coordonnée en degrés-minutes
  - `direction` (str): Direction ('N', 'S', 'E', 'W') pour appliquer le signe correct
- **Output**: 
  - `dd` (float) - Coordonnée en degrés décimaux (DD)

---

### `dms_to_dd(dms, direction)`:
Convertit les coordonnées au format degrés, minutes, secondes (DMS) vers le format degrés décimaux (DD).
- **Input**:
  - `dms` (str): Coordonnée au format degrés, minutes, secondes (DMS) sous forme de chaîne
  - `direction` (str): Direction ('N', 'S', 'E', 'W') pour appliquer le signe correct
- **Output**: 
  - `dd` (float) - Coordonnée en degrés décimaux (DD)

---

### `mesure_gps(fichier="/mesures/gps_data.txt")`:
Récupère les données GPS (GLL) et les écrit dans un fichier.
- **Input**:
  - `fichier` (str): Chemin du fichier où les données GPS doivent être enregistrées
- **Output**: 
  - `gll_data` (tuple) - Données GPS sous forme de tuple

---

### `create_csv(input_file, output_csv_path)`:
Convertit un fichier de données GPS (.txt) en un fichier CSV avec latitude et longitude.
- **Input**:
  - `input_file` (str): Chemin du fichier .txt contenant les données GPS
  - `output_csv_path` (str): Chemin du fichier CSV de sortie
- **Output**: Aucun

---

### `afficher_data(csv_file, output_geojson_file)`:
Crée un fichier GeoJSON à partir des données GPS contenues dans un fichier CSV.
- **Input**:
  - `csv_file` (str): Chemin du fichier CSV contenant les données GPS
  - `output_geojson_file` (str): Chemin du fichier GeoJSON de sortie
- **Output**: Aucun

---

### `projection(lat, long, lat_m=48.199170, long_m=-3.014700)`:
Convertit les coordonnées GPS (latitude, longitude) en coordonnées cartésiennes locales par rapport à un point de référence.
- **Input**:
  - `lat` (float): Latitude en degrés décimaux
  - `long` (float): Longitude en degrés décimaux
  - `lat_m` (float): Latitude du point de référence en degrés décimaux (optionnel)
  - `long_m` (float): Longitude du point de référence en degrés décimaux (optionnel)
- **Output**: 
  - `p` (np.array) - Coordonnées cartésiennes relatives [x, y]

---

### `cap_waypoint(a, p)`:
Calcule le cap à suivre pour atteindre un waypoint depuis un point de départ.
- **Input**:
  - `a` (np.array): Coordonnée du point de départ [latitude, longitude]
  - `p` (np.array): Coordonnée du waypoint [latitude, longitude]
- **Output**: 
  - `cap` (float) - Cap à suivre en radians

### `arret_waypoint(a, p, distance_min=2)`:
Vérifie si la distance entre le point actuel et le waypoint est inférieure à une distance minimale spécifiée, pour déclencher l'arrêt.

- **Input**:
  - `a` (np.array): Coordonnées actuelles [x, y]
  - `p` (np.array): Coordonnées du waypoint [x, y]
  - `distance_min` (float): Distance minimale pour considérer l'arrêt (par défaut 2)

- **Output**: 
  - `True` si la distance est inférieure à `distance_min`, sinon `False`

---

### `regulation_vitesse(distance, vmax=200, vmin=45, coef=1, middle=4)`:
Régule la vitesse en fonction de la distance à un point de référence. La vitesse est calculée à l'aide d'une fonction sigmoïde pour assurer une transition fluide.

- **Input**:
  - `distance` (float): Distance actuelle à un point de référence
  - `vmax` (int): Vitesse maximale (par défaut 200)
  - `vmin` (int): Vitesse minimale (par défaut 45)
  - `coef` (float): Coefficient de régulation (par défaut 1)
  - `middle` (float): Distance centrale pour la régulation (par défaut 4)

- **Output**:
  - `vitesse` (float): Vitesse régulée entre `vmin` et `vmax`

---

### `reach_point()`
Rejoindre un point GPS donné par le point A en format degrés décimaux.

- **Input**: 
  - `lat_a` (float): Latitude du point A.
  - `long_a` (float): Longitude du point A.
  - `debug` (bool, optionnel): Affiche les messages de débogage si `True`. Par défaut `True`.

- **Output**: `None`

---

### `cap_waypoint_2()`
Calcule le cap pour suivre une droite définie par un point A et un vecteur directeur n, depuis un point P.

- **Input**: 
  - `m` (np.array): Point A.
  - `n` (np.array): Vecteur directeur de la droite.
  - `p` (np.array): Point P.
  - `debug` (bool, optionnel): Affiche les messages de débogage si `True`. Par défaut `False`.

- **Output**: 
  - `cap_d` (float): Cap calculé en radians.

---

### `suivre_droite()`
Permet de suivre une droite définie par un point M et un point A, exprimés en format degrés décimaux.

- **Input**: 
  - `M` (list): Coordonnées GPS du point de départ `[lat, long]`.
  - `A` (list): Coordonnées GPS du point d’arrivée `[lat, long]`.
  - `debug` (bool, optionnel): Affiche les messages de débogage si `True`. Par défaut `True`.

- **Output**: `None`

---

### `distance_droite()`
Calcule la distance entre un point P et une droite définie par un point A et un vecteur normal n.

- **Input**: 
  - `a` (np.array): Point A sur la droite.
  - `n` (np.array): Vecteur normal à la droite.
  - `p` (np.array): Point P.

- **Output**: 
  - `distance` (float): Distance entre le point P et la droite. La distance est positive si P est à gauche de la droite dans le sens du vecteur n.

