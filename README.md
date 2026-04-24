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

## Schnelleinstieg

### 1. Frontend - Flutter App

```bash
cd speedpilot\frontend

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
cd speedpilot/ros_backend

# Docker Image bauen
cd docker
bash build.sh

# Container starten (docker-compose)
docker compose up

# Oder manuell starten
docker run -it --privileged --network host \
  -v $(pwd)/../ros2_ws:/root/ros2_ws \
  speedpilot:latest bash
```

**Abhängigkeiten:** Docker 24+, Docker Compose v2

### 3. Nodes starten

```bash
# Im laufenden Container
docker exec -it speedpilot bash

# WebSocket-Bridge starten (Port 9091)
ros2 run ros2_bridge bridge_node

# Fahrzeugsteuerung starten
ros2 run car_controller controller_node

# Ultraschallsensor starten
ros2 run ultrasonic_sensor ultrasonic_node

# Hindernisvermeidung starten (optional)
ros2 run lidar_obstacle_avoidance obstacle_avoidance_node
```

### 4. Kommunikation testen

```bash
# Im Backend-Container: Fahrbefehl manuell senden
ros2 topic pub /vehicle_command custom_msgs/msg/VehicleCommand \
  "{command: 'move', speed: 0.5, angle: 0.0}"

# Befehl stoppen
ros2 topic pub /vehicle_command custom_msgs/msg/VehicleCommand \
  "{command: 'stop', speed: 0.0, angle: 0.0}"

# Ultraschalldistanz überwachen
ros2 topic echo /ultrasonic/distance
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
