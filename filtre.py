import numpy as np

class SensorFilter:
    def __init__(self, window_size=5):
        """
        Initialise un filtre de moyenne glissante.
        
        Paramètres :
            window_size (int) : Nombre de valeurs utilisées pour lisser les mesures.
        """
        self.window_size = window_size
        self.values_x = []
        self.values_y = []
        self.values_z = []


    def update_x(self, new_value):
        """
        Ajoute une nouvelle mesure et retourne la valeur filtrée.
        
        Paramètres :
            new_value (float) : Nouvelle valeur du capteur.
        
        Retour :
            float : Valeur filtrée.
        """
        self.values_x.append(new_value)
        if len(self.values_x) > self.window_size:
            self.values_x.pop(0)  # Supprime la plus ancienne valeur
        
        return np.mean(self.values_x)  # Retourne la moyenne des valeurs stockées
    
    def update_y(self, new_value):
        """
        Ajoute une nouvelle mesure et retourne la valeur filtrée.
        
        Paramètres :
            new_value (float) : Nouvelle valeur du capteur.
        
        Retour :
            float : Valeur filtrée.
        """
        self.values_y.append(new_value)
        if len(self.values_y) > self.window_size:
            self.values_y.pop(0)  # Supprime la plus ancienne valeur
        
        return np.mean(self.values_y)  # Retourne la moyenne des valeurs stockées
    
    def update_z(self, new_value):
        """
        Ajoute une nouvelle mesure et retourne la valeur filtrée.
        
        Paramètres :
            new_value (float) : Nouvelle valeur du capteur.
        
        Retour :
            float : Valeur filtrée.
        """
        self.values_z.append(new_value)
        if len(self.values_z) > self.window_size:
            self.values_z.pop(0)  # Supprime la plus ancienne valeur
        
        return np.mean(self.values_z)  # Retourne la moyenne des valeurs stockées