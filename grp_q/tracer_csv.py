import matplotlib.pyplot as plt
import pandas as pd

"""
Ce fichier permet de tracer un graphique à partir d'un fichier .csv
contenant deux colonnes X et Y
"""

def tracer_gps_from_csv(filename):
    # Lire le fichier CSV
    data = pd.read_csv(filename)
    
    # Assure-toi que les colonnes X et Y existent
    if 'X' not in data.columns or 'Y' not in data.columns:
        print("Le fichier CSV ne contient pas les colonnes 'X' et 'Y'.")
        return

    # Extraire les colonnes X et Y
    x = data['X']
    y = data['Y']
    
    # Créer le graphique
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, marker='o', linestyle='-', color='b', label='Coordonnées GPS')
    plt.title('Projection GPS')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.show()

# Exemple d'utilisation
tracer_gps_from_csv('points_projection.csv')
