import csv
import json
import os
import ast

"""
Ce fichier permet à partir d'un fichier txt contenant les données gps du capteurs de 
retourner un fichier geojson
"""

filename = 'gps_data.txt'
csv_file = 'gps_data.csv'
output_geojson_file = 'gps_data.geojson'

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


def afficher_data(csv_file, output_geojson_file):
    geojson_data = {
        "type": "FeatureCollection",
        "features": []
    }

    try:
        with open(csv_file, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                try:
                    latitude = float(row['Latitude'])
                    longitude = float(row['Longitude'])
                    
                    feature = {
                        "type": "Feature",
                        "geometry": {
                            "type": "Point",
                            "coordinates": [longitude, latitude]  # GeoJSON [long, lat]
                        },
                        "properties": {}
                    }
                    geojson_data["features"].append(feature)
                except ValueError:
                    print(f"Erreur de conversion dans la ligne : {row}")
                    
        with open(output_geojson_file, 'w') as geojson_file:
            json.dump(geojson_data, geojson_file, indent=4)
        print(f"Fichier GeoJSON généré avec succès : {output_geojson_file}")
    except FileNotFoundError:
        print(f"Erreur : fichier {csv_file} introuvable.")
    except Exception as e:
        print(f"Erreur : {e}")

# Exécuter les fonctions
create_csv(filename, csv_file)
afficher_data(csv_file, output_geojson_file)
