import json
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

# Konstanten für Plot-Anpassungen
RED = 'red'
BLUE = 'blue'
PLOT_SIZE = (8, 8)
POINT_SIZE = 10
CENTER_POINT_SIZE = 50
TITLE = "LIDAR Datenvisualisierung"
PLOT_LABEL = 'Mittelpunkt'

BASE_DIR = Path(__file__).resolve().parent
LIDAR_DATA_PATH = BASE_DIR / "assets" / "lidar_data" / "lidar_scan_5.json"
OUTPUT_IMAGE_PATH = BASE_DIR / "assets" / "images" / "lidar.jpg"


def load_lidar_data(file_path):
    """
    Lädt LIDAR-Daten aus einer JSON-Datei.
    :param file_path: Pfad zur JSON-Datei
    :return: Dictionary mit LIDAR-Daten
    """
    try:
        with open(file_path, "r") as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"Fehler: Datei {file_path} wurde nicht gefunden.")
        return None
    except json.JSONDecodeError:
        print("Fehler: Die JSON-Datei konnte nicht decodiert werden.")
        return None


def generate_angles(min_angle, max_angle, angle_increment):
    """
    Generiert eine Liste von Winkeln basierend auf den LIDAR-Daten.
    :param min_angle: Minimale Winkelgrenze
    :param max_angle: Maximale Winkelgrenze
    :param angle_increment: Winkel-Inkrement
    :return: Liste von Winkeln
    """
    angles = []
    current_angle = min_angle
    while current_angle <= max_angle:
        angles.append(current_angle)
        current_angle += angle_increment
    return angles


def create_dataframe(angles, distances):
    """
    Erstellt ein DataFrame aus den Winkeln und Distanzen.
    :param angles: Liste der Winkel
    :param distances: Liste der Distanzen
    :return: Pandas DataFrame
    """
    return pd.DataFrame({
        "Winkel": angles,
        "Distance": distances
    })


def plot_polar_data(angles, distances):
    """
    Erzeugt einen Polarplot mit den gegebenen Winkeln und Distanzen.
    :param angles: Liste der Winkel
    :param distances: Liste der Distanzen
    """
    plt.figure(figsize=PLOT_SIZE)
    ax = plt.subplot(111, projection='polar')
    ax.scatter(angles, distances, c=BLUE, s=POINT_SIZE)
    ax.scatter(0, 0, c=RED, s=CENTER_POINT_SIZE, label=PLOT_LABEL)
    ax.set_title(TITLE)
    ax.legend(loc='upper right')
    OUTPUT_IMAGE_PATH.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(OUTPUT_IMAGE_PATH)


def main():
    # LIDAR-Daten laden
    lidar_data = load_lidar_data(LIDAR_DATA_PATH)
    
    if lidar_data is None:
        return  

    min_angle = lidar_data["angle_min"]
    max_angle = lidar_data["angle_max"]
    angle_increment = lidar_data["angle_increment"]
    ranges = lidar_data["ranges"]

    angles = generate_angles(min_angle, max_angle, angle_increment)
    distances = ranges[:len(angles)] 

    df = create_dataframe(angles, distances)
    print(df)

    plot_polar_data(df["Winkel"], df["Distance"])


if __name__ == "__main__":
    main()
