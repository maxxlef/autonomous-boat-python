from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
from numpy import *
#from quoicouroblib import *


def f(x,u):
    x,u=x.flatten(),u.flatten()
    xdot = array([[x[3]*cos(x[2])],[x[3]*sin(x[2])],[u[0]],[u[1]]])
    return(xdot)

def control_cap(x, waypoint, k_theta=1.0):
    """
    Contrôle le cap pour orienter le tank vers le waypoint.

    Arguments:
    - x : état du tank (position, cap, vitesse)
    - waypoint : coordonnée cible [x_w, y_w]
    - k_theta : gain proportionnel pour la correction du cap

    Retourne:
    - u_0 : commande de rotation (rad/s)
    """
    theta = x[2, 0]  # Cap actuel du tank
    x_tank, y_tank = x[0, 0], x[1, 0]  # Position actuelle
    x_w, y_w = waypoint[0], waypoint[1]  # Coordonnées du waypoint

    # Calcul du cap désiré vers le waypoint
    theta_d = arctan2(y_w - y_tank, x_w - x_tank)

    # Calcul de l'erreur angulaire avec un angle entre -pi et pi
    err_theta = arctan2(sin(theta_d - theta), cos(theta_d - theta))

    # Commande proportionnelle
    u_0 = k_theta * err_theta
    return u_0

def force_attract_repel_sequential(x, X, i, d_min=40, d_max=50, k_rep=0.5, k_att=0.5):
    """
    Force d'attraction uniquement vers le bateau précédent (i-1) et répulsion contre tous.
    
    Paramètres :
    - x : état du bateau considéré (position x, y, cap, vitesse)
    - X : Matrice des états de tous les bateaux (4 x m)
    - i : Indice du bateau actuel dans X
    - d_min : Distance minimale avant répulsion
    - d_max : Distance maximale avant attraction
    - k_rep : Gain pour la répulsion
    - k_att : Gain pour l'attraction

    Retourne :
    - correction_u0 : Ajustement de l'angle de direction
    - correction_u1 : Ajustement de la vitesse
    """
    correction_u0 = 0
    correction_u1 = 0

    for j in range(X.shape[1]):  # Parcours des autres bateaux
        if j == i:  
            continue  # Ignore le bateau lui-même

        x_other, y_other = X[0, j], X[1, j]
        distance = np.linalg.norm([x_other - x[0, 0], y_other - x[1, 0]])

        if distance == 0:
            continue  # Évite la division par zéro

        angle_to_other = np.arctan2(y_other - x[1, 0], x_other - x[0, 0])
        err_angle = np.arctan2(np.sin(angle_to_other - x[2, 0]), np.cos(angle_to_other - x[2, 0]))

        # Répulsion contre tous les bateaux trop proches
        if distance < d_min:  
            force = -k_rep * (d_min - distance)
            correction_u0 += force * err_angle  # Influence l'angle de direction
            correction_u1 += force * np.cos(err_angle)  # Influence la vitesse

        # Attraction uniquement vers le bateau `i-1`
        elif j == (i - 1) % X.shape[1] and distance > d_max:
            force = k_att * (distance - d_max)
            correction_u0 += force * err_angle
            correction_u1 += force * np.cos(err_angle)

    return correction_u0, correction_u1


def controle_vit(x,waypoints,k_accel=1.0):
    vit = x[3,0] # vitesse actuelle
    x_tank, y_tank = x[0, 0], x[1, 0]
    x_w, y_w = waypoints[0], waypoint[1]
    d = np.array([[x_tank],[y_tank]]) - np.array([[x_w],[y_w]])
    distance = np.linalg.norm(d)
    vmax=5
    vmin=0
    coef=1
    middle=20
    vitesse_d = (vmax-vmin) * (0.5 * (1 - np.tanh(coef * (middle - distance)))) + vmin #vitesse voulue
    u_1=k_accel*(vitesse_d-vit)
    return u_1
 

ax=init_figure(-100,100,-100,100)

m   = 6
X   = 60*randn(4,m)

"""
m = 6   # Nombre de tanks
rayon = 50  # Rayon du cercle
angles = np.linspace(0, 2*np.pi, m, endpoint=False)  # Angles pour placer les tanks

X = np.zeros((4, m))  # Matrice pour stocker les états des tanks

for i in range(m):
    X[0, i] = rayon * np.cos(angles[i])  # Coordonnée x
    X[1, i] = rayon * np.sin(angles[i])  # Coordonnée y
    X[2, i] = 0 # Cap du tank (orienté vers l'intérieur)
    X[3, i] = 0  # Vitesse initiale à 0
"""

a,dt = 0.1,0.1

lat_a,long_a = np.random.uniform(0, 50, size=(2, 1))  # target

for t in arange(0,100,dt):
    clear(ax)
    for i in range(m):        

        x=X[:,i].reshape(4,1)

        #u = reach_point_simu(lat_a,long_a,x) a faire marcher

        #waypoint = np.array([lat_a, long_a])  # Convertir en liste [x_w, y_w]
        if i==0:
            waypoint= np.array([[float(X[0,-1])], [float(X[0,-1])]])
        else :
            waypoint= np.array([[float(X[0,i-1])], [float(X[0,i-1])]])
        
        u_0 = control_cap(x, waypoint)  # Calcul de la rotation

        u_1 = controle_vit(x,waypoint) # calcul de l'accel

        correction_u0, correction_u1 = force_attract_repel_sequential(x, X,i)

        u_0 += 0.2*correction_u0

        u_1 += 0.2*correction_u1

        u = np.array([[float(u_0)], [float(u_1)]])  # Construire le vecteur de commande

        x=X[:,i].reshape(4,1)

        draw_tank(x,'b')

        x=x+f(x,u)*dt

        X[:,i]  = x.flatten()



