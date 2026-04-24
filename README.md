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
cd speedpilot\ros_backend\docker

# Docker Image bauen
cd docker
bash build.sh

# Container starten (docker-compose)
docker compose up

# Im Container arbeiten (nur gpiochip4 ist essentiell)
docker run -it   --device /dev/gpiochip0 --device /dev/gpiochip1 --device /dev/gpiochip2 --device /dev/gpiochip3 --device /dev/gpiochip4 --privileged ImageID bash

# ROS 2 System starten
bash 
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch speedpilot_backend bringup.launch.py

------------
# Patch 
colcon build
source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 src/car_system_launch.py

# Save Docker Image for offline capability
docker save -o ros2_speedpilot_backend.tar ImageName

```

**Abhängigkeiten:** ROS 2 Jazzy, Python 3.12, Docker, CMake 3.28+

### 3. Kommunikation testen

```bash
# Im Backend-Container: Fahrbefehl manuell senden
ros2 topic pub /vehicle_command custom_msgs/msg/VehicleCommand \
  "{command: 'move', speed: 0.5, angle: 0.0}"

# Verfügbare Befehle: FORWARD, LEFT, RIGHT, STOP, BACKWARD
```

### 4. Schnellstart auf Pi nach initialem Builden und speichern des Docker Images:
```bash
docker load -i ros2_speedpilot_backend.tar
docker compose up
```

---

## TODOs

### Code Umschreibung
Das migrieren von Ubuntu Desktop (Version 23.x) zu Ubuntu Server (24.x LTS Version) führte dazu, dass die GPIO Pins nicht mehr bekannt sind.

Der Code muss umgeschrieben werden, um statt der Library RPi.GPIO gpiod verwendet werden:
```python
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE=FALSE
```
muss umgeschrieben werden zu:
```python
try:
    import gpiod
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
```

#### Wir sollten hauptsächlich gpiochip4 verwenden, die anderen GPIo's sind USB usw.

RPi.GPIO -> libgpiod mapping

| RPi.GPIO | libgpiod |
|----------|----------|
|GPIO.setmode(GPIO.BCM) ≤ Not needed - libgpiod uses chip + line numbers |
| GPIO.setup(pin, GPIO.OUT) | line = chip.get_line(pin); line.request(OUTPUT) |
| GPIO.output(pin, GPIO.HIGH) | line.set_value(1)
| GPIO.cleanup() | line.release() |

Als Beispielcode:
#### Alt:
```python
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

GPIO.output(17, GPIO.HIGH)
GPIO.output(17, GPIO.LOW)

GPIO.cleanup()
```

#### Neu:

```python
import gpiod
import time

chip = gpiod.Chip("gpiochip4")   # use gpiochip4
line = chip.get_line(17)         # BCM pin number

config = gpiod.LineRequest()
config.consumer = "car_controller"
config.request_type = gpiod.LINE_REQ_DIR_OUT

line.request(config)

line.set_value(1)
time.sleep(1)
line.set_value(0)

line.release()
chip.close()

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
