# Speedpilot - Autonomous Vehicle System

**Studienarbeit DHBW Ravensburg 2025**

Ein vollständiges autonomes Fahrzeugsystem bestehend aus einer Flutter-Mobilitäts-App (Frontend) und ROS 2 Backend mit WebSocket-Kommunikation, Sensorintegration und Echtzeit-Fahrzeugsteuerung.

---

## Überblick

**Speedpilot** ist ein System zur Fernsteuerung eines Raspberry-Pi-basierten Fahrzeugs mit:
- **Frontend**: Flutter App für Joystick/Gyroskop-Steuerung
- **Backend**: ROS 2 mit Echtzeit-Befehlsausführung
- **Kommunikation**: WebSocket-basierte Bidirektional-Kommunikation
- **Sensoren**: LIDAR-Integration für Kartenerstellung & Hindernisvermeidung
- **Deployment**: Docker-containerisiert für einfache Skalierung

---

## Projektstruktur

```
Studienarbeit-DHBW2025/
│
├── speedpilot-main/
│   ├── flutter/                      # Flutter Build-Konfigurationen
│   │   ├── distributions/            # OS-spezifische Builds
│   │   │   ├── android/
│   │   │   ├── ios/
│   │   │   ├── macos/
│   │   │   ├── windows/
│   │   │   ├── linux/
│   │   │   └── web/
│   │   └── .dart_tool/, .metadata, .fvmrc
│   ├── lib/                          # Dart-Quellcode (plattformunabhängig)
│   │   ├── main.dart                 # Einstiegspunkt
│   │   ├── driving_page/             # Fahrzeugsteuerung
│   │   ├── map_page/                 # Kartendarstellung
│   │   ├── settings_page/            # Einstellungen
│   │   ├── starting_page/            # Verbindungssetup
│   │   ├── models/                   # Datenmodelle
│   │   └── services/                 # WebSocket, APIs
│   ├── assets/                       # Bilder & LIDAR-Datensätze
│   └── pubspec.yaml                  # Flutter Dependencies
│
├── speedpilot_ros_backend-main/
│   ├── ros2_ws/
│   │   └── src/
│   │       ├── car_controller/       # Fahrzeugsteuerung
│   │       ├── ros2_bridge/          # WebSocket Bridge
│   │       ├── custom_msgs/          # Nachrichtendefinitionen
│   │       └── ultrasonic_sensor/    # Sensorintegration
│   ├── docker/
│   │   ├── Dockerfile                # Container-Image
│   │   └── docker-compose.yml        # Orchestration
│   └── workspace.sh                  # Setup-Skript
│
├── Studienarbeit/
│   ├── main.tex                      # LaTeX Hauptdatei
│   ├── Grundlagen.tex
│   ├── Konzept und Anforderungsdefinition.tex
│   ├── Implementierung.tex
│   ├── Evaluation und Validierung.tex
│   └── literatur/                    # Quellen
│
└── README.md                         # (Diese Datei)
```

---

## Schnelleinstieg

### 1. Frontend - Flutter App

```bash
cd speedpilot-main

# Abhängigkeiten installieren
flutter pub get

# iOS Simulator (macOS only)
open -a Simulator
flutter run

# Android Emulator
flutter emulators --launch <emulator_id>
flutter run

# Echtes Gerät
flutter devices        # Gerät auflisten
flutter run           # Auf angeschlossenem Gerät ausführen
```

**Struktur:** `lib/` → plattformunabhängiger Code | `flutter/distributions/` → OS-spezifische Builds

### 2. Backend - ROS 2 + Docker

```bash
cd speedpilot_ros_backend-main

# Docker Image bauen & Container starten
docker compose up

# Im Container arbeiten
docker compose exec manipulation bash

# ROS 2 System starten
ros2 launch speedpilot_backend bringup.launch.py
```

**Abhängigkeiten:** ROS 2 Jazzy, Python 3.12, Docker, CMake 3.28+

### 3. Kommunikation testen

```bash
# Im Backend-Container
ros2 topic pub /vehicle/cmd std_msgs/String "data: 'FORWARD'"

# Verfügbare Befehle: FORWARD, LEFT, RIGHT, STOP, BACKWARD
```

---

## Features & Komponenten

### Frontend (Flutter)
| Feature | Beschreibung |
|---------|-------------|
| **Joystick-Steuerung** | Echtzeitsteuerung mit visueller Rückmeldung |
| **Gyroskop-Steuerung** | Geräteorientierung für intuitive Navigation |
| **LIDAR-Visualisierung** | Occupancy Grid + Polarplot-Darstellung |
| **Kartenvisualisierung** | Scroll/Zoom mit mehreren Kartenebenen |
| **WebSocket** | Bidirektionale Echtzeit-Kommunikation |

### Backend (ROS 2)
| Komponente | Beschreibung |
|-----------|-------------|
| **car_controller** | Fahrzeugbefehlsausführung |
| **ros2_bridge** | WebSocket-Server zum App-Backend |
| **custom_msgs** | ROS 2 Nachrichtendefinitionen |
| **Sensoren** | Ultraschall, LIDAR-Integration |
| **Hindernisvermeidung** | Automatische Kollisionsvermeidung |

---

## Technologie-Stack

### Frontend
```
Flutter 3.27.0 | Dart 3.6.1
├── flutter_joystick: Steuerung
├── flutter_rviz: ROS-Visualisierung
├── web_socket_channel: Kommunikation
├── sensors_plus: Sensorenzugriff
└── shared_preferences: Persistenter Speicher
```

### Backend
```
ROS 2 Jazzy
├── Python 3.12.3
├── C++ 13.3.0
├── CMake 3.28.3
└── Docker + Docker Compose
```

---

## Zusätzliche Ressourcen

- **Studienarbeit-Dokumentation**: [main.tex](Studienarbeit/main.tex)
- **ROS 2 Dokumentation**: https://docs.ros.org/en/jazzy/
- **Flutter Dokumentation**: https://flutter.dev/docs

---

## Related Projects

- **[Speedpilot Frontend](https://github.com/tobiassng/speedpilot)** - Original Mobile App
- **[Speedpilot Backend](https://github.com/Rufffy99/speedpilot_ros_backend)** - ROS 2 Backend

---

## Über diese Studienarbeit

Diese Arbeit wurde an der **DHBW Ravensburg** durchgeführt und beschreibt die Entwicklung eines vollständigen Fahrzeugsystems mit:
- Anforderungsdefinition & Systemkonzept
- Frontend-Implementierung in Flutter
- Backend-Implementierung in ROS 2
- Sensor-Integration & Echtzeitkommunikation
- Umfassende Evaluation & Validierung

**Zeitraum:** 2025 | **Universität:** DHBW Ravensburg

---

## Lizenz

Dieses Projekt ist Teil einer Studienarbeit und unterliegt den akademischen Richtlinien der DHBW Ravensburg.
