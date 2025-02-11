import numpy as np
import matplotlib.pyplot as plt
import time

def cercle(t, lat_boue, long_boue, k=0):
    """
    Calcule la position virtuelle p_tilde en fonction du temps t.
    
    Input:
        - t : temps en secondes
        - lat_boue, long_boue : coordonnées du point de départ
        - k : déphasage optionnel (par défaut 0)
    
    Output:
        - p_tilde : coordonnées simulées de la trajectoire virtuelle
    """
    r = 10  # rayon du cercle (mètres)
    T = 100  # période en secondes
    m = np.array([[lat_boue], [long_boue]])

    # Calcul de la position virtuelle p_tilde
    p_tilde = m + r * np.array([
        [np.cos(2 * np.pi * ((t) / T + k / 10))],
        [np.sin(2 * np.pi * ((t) / T + k / 10))]
    ])
    return p_tilde

def tracer_trajectoire(lat_boue=0, long_boue=0, durée=500):
    """
    Trace la trajectoire virtuelle p_tilde sur une durée donnée.
    
    Input:
        - lat_boue, long_boue : coordonnées initiales
        - durée : durée de la simulation en secondes
    """
    plt.figure(figsize=(10, 10))
    plt.xlim(-20, 20)
    plt.ylim(-20, 20)
    plt.grid(True)
    plt.xlabel("Latitude simulée")
    plt.ylabel("Longitude simulée")
    plt.title("Trajectoire virtuelle de p_tilde")
    
    # Initialisation du temps
    t0 = time.time()
    trajectoire_x, trajectoire_y = [], []
    
    while time.time() - t0 < durée:
        t = time.time() - t0
        p_tilde = cercle(t, lat_boue, long_boue)
        trajectoire_x.append(p_tilde[0, 0])
        trajectoire_y.append(p_tilde[1, 0])
        
        plt.cla()
        plt.xlim(-20, 20)
        plt.ylim(-20, 20)
        plt.grid(True)
        plt.plot(trajectoire_x, trajectoire_y, 'b-', label="Trajectoire virtuelle")
        plt.scatter(p_tilde[0, 0], p_tilde[1, 0], color='red', label="p_tilde (actuel)")
        plt.legend()
        plt.pause(0.01)
    
    plt.show()

# Exécuter la simulation
tracer_trajectoire()
