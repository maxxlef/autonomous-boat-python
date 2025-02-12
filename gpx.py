import csv
from datetime import datetime, timedelta
import csv
import json
import os
import ast

"""
Ce fichier permet à partir d'un fichier txt contenant les données gps du capteurs de 
retourner un fichier gpx
"""

filename = 'gps_data.txt'
csv_file = 'gps_data.csv'
output_gpx_file = 'gps_data.gpx'

def dm_to_dd(dm,direction):
    degrees = int(dm // 100)
    minutes = dm - degrees * 100
    dd = degrees + (minutes / 60)
    if direction in ['S', 'W']:
        dd *= -1
    return dd

def create_csv(input_file, output_csv_path):
    try:
        with open(input_file, 'r') as file:
            file_content = file.read()

        with open(output_csv_path, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['Latitude', 'Longitude'])
            
            for line in file_content.strip().split('\n'):
                try:
                    data = ast.literal_eval(line.strip())  # Plus sûr que eval
                    latitude_dm, lat_dir, longitude_dm, long_dir = data[:4]
                    print("Latitude: {}, Direction: {}".format(latitude_dm, lat_dir))
                    print("Longitude: {}, Direction: {}".format(longitude_dm, long_dir))
                    latitude = dm_to_dd(latitude_dm, lat_dir)
                    longitude = dm_to_dd(longitude_dm, long_dir)
                    
                    if latitude is not None and longitude is not None:
                        csv_writer.writerow([latitude, longitude])
                except (SyntaxError, ValueError) as e:
                    print("Erreur lors du traitement de la ligne : {}".format(line))
        print("Fichier CSV généré avec succès : {}".format(output_csv_path))
    except FileNotFoundError:
        print("Erreur : fichier {} introuvable.".format(input_file))
    except Exception as e:
        print("Erreur : {}".format(e))


def create_gpx(csv_file, output_gpx_path, start_time="2025-02-12T08:00:00Z"):
    try:
        with open(csv_file, 'r') as file:
            reader = csv.DictReader(file)
            gpx_content = """<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="MonScript" xmlns="http://www.topografix.com/GPX/1/1">
"""
            # Convertir la chaîne de départ en objet datetime
            current_time = datetime.fromisoformat(start_time.replace("Z", "+00:00"))
            
            for row in reader:
                try:
                    latitude = float(row['Latitude'])
                    longitude = float(row['Longitude'])
                    # Ajouter un waypoint avec une balise <time>
                    gpx_content += f"""  <wpt lat="{latitude}" lon="{longitude}">
    <name>Point</name>
    <time>{current_time.isoformat()}Z</time>
  </wpt>
"""
                    # Incrémenter le temps d'une seconde pour le prochain point
                    current_time += timedelta(seconds=1)
                except ValueError:
                    print(f"Erreur de conversion dans la ligne : {row}")
                    
            gpx_content += "</gpx>"

        with open(output_gpx_path, 'w') as gpx_file:
            gpx_file.write(gpx_content)
        print(f"Fichier GPX généré avec succès : {output_gpx_path}")
    except FileNotFoundError:
        print(f"Erreur : fichier {csv_file} introuvable.")
    except Exception as e:
        print(f"Erreur : {e}")


# Exécuter les fonctions
create_csv(filename, csv_file)
create_gpx(csv_file, output_gpx_file)