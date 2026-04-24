# SpeedPilot – Flutter Frontend

Flutter-App zur Fernsteuerung des SpeedPilot-Fahrzeugs. Unterstützt iOS, Android und Windows.

## Voraussetzungen

- Flutter 3.27.0+ / Dart 3.6.1+
- Android SDK oder Xcode (für iOS)
- Das ROS 2 Backend muss laufen und erreichbar sein (Port 9091)

## Setup

```bash
flutter pub get
```

## Starten

```bash
# Verbundenes Gerät/Emulator auflisten
flutter devices

# Auf gewünschtem Gerät starten
flutter run

# Release-Build für Android
flutter build apk --release

# Release-Build für iOS (macOS erforderlich)
flutter build ios --release
```

## Verbindung zum Fahrzeug

1. App starten
2. IP-Adresse des Raspberry Pi eingeben (Fahrzeug und Smartphone müssen im selben WLAN sein)
3. Verbinden – die App öffnet eine WebSocket-Verbindung zu `ws://<IP>:9091`
4. Steuerungsmodus wählen (Joystick oder Gyroskop)

Gespeicherte Geräte werden über `shared_preferences` persistent gespeichert.

## Steuerungsmodi

| Modus       | Beschreibung                                                                 |
|------------|------------------------------------------------------------------------------|
| Joystick    | Linker Joystick = Gas/Bremse, rechter Joystick = Lenkung                    |
| Gyroskop    | Smartphone-Neigung steuert Geschwindigkeit (Pitch) und Lenkung (Roll)       |

Umschalten über die Einstellungsseite (⚙).

## Projektstruktur

```
frontend/
├── lib/
│   ├── main.dart
│   ├── starting_page/       Geräteauswahl und Verbindungsaufbau
│   ├── map_page/            Kartenansicht mit Modusauswahl
│   ├── driving_page/        Steuerungsseiten (Joystick + Gyroskop)
│   │   └── features/
│   │       ├── tachometer.dart         Geschwindigkeitsanzeige
│   │       ├── steering_joystick.dart  Lenkjoystick
│   │       ├── gas_joystick.dart       Gas-/Bremsjoystick
│   │       ├── lidar_data.dart         LiDAR-Polarplot
│   │       ├── gyroscope_data.dart     Gyroskop-Steuerlogik
│   │       └── occupancy_grid.dart     Belegungsgitter-Anzeige
│   ├── settings_page/       Einstellungen (Steuerungsmodus)
│   ├── models/              Datenmodelle
│   └── services/
│       └── WebSocketManager.dart   WebSocket-Verbindungsmanagement
├── lidar/
│   ├── lidar_data.py         Python-Hilfsskript zur LiDAR-Visualisierung
│   └── assets/lidar_data/    Beispiel-LiDAR-Scandaten (JSON)
└── flutter/distributions/    Plattformspezifische Build-Konfigurationen
```

## Abhängigkeiten (pubspec.yaml)

| Paket                       | Verwendung                          |
|-----------------------------|-------------------------------------|
| `web_socket_channel`        | WebSocket-Kommunikation mit Bridge  |
| `flutter_joystick`          | Joystick-Widget                     |
| `syncfusion_flutter_gauges` | Tachometer-Anzeige                  |
| `flutter_rviz`              | ROS-Visualisierung (OccupancyGrid)  |
| `sensors_plus`              | Gerätebeschleunigungssensor         |
| `shared_preferences`        | Persistente Gerätespeicherung       |
