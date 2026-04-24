# SpeedPilot

Ein vollständiges Fahrzeugsteuerungssystem bestehend aus einer Flutter-App (Frontend) und einem ROS 2 Backend (ros_backend), die über WebSocket kommunizieren.

```
Mobile App  ──WebSocket:9091──▶  ros2_bridge  ──▶  vehicle_command topic
                                                         │
                                              ┌──────────┴──────────┐
                                         car_controller    lidar_obstacle_avoidance
                                              │
                                          GPIO PWM
                                         Motor + Servo
```

## Voraussetzungen

| Komponente      | Version         |
|----------------|-----------------|
| Flutter / Dart | 3.27.0 / 3.6.1  |
| Docker         | 24+             |
| Docker Compose | v2              |
| Raspberry Pi   | 4 (64-bit OS)   |
| ROS 2          | Jazzy (via Docker) |

---

## Quickstart

### Frontend

```bash
cd frontend
flutter pub get
flutter run          # Gerät/Emulator muss verbunden sein
```

### Backend (lokal, für Entwicklung)

```bash
cd ros_backend/docker
bash build.sh        # Image bauen: speedpilot:latest
docker compose up    # Container starten
```

---

## Raspberry Pi Deployment

### 1. Raspberry Pi vorbereiten

Raspberry Pi OS 64-bit (Bookworm) empfohlen. Auf dem Pi ausführen:

```bash
# System aktualisieren
sudo apt update && sudo apt upgrade -y

# Docker installieren
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
newgrp docker

# Docker Compose Plugin installieren (falls nicht enthalten)
sudo apt install -y docker-compose-plugin

# Prüfen
docker --version
docker compose version
```

### 2. Code auf den Pi übertragen

**Option A – Git Clone (empfohlen):**

```bash
# Auf dem Pi
git clone <repository-url> speedpilot
cd speedpilot/ros_backend
```

**Option B – SCP (von der Entwicklungsmaschine):**

```bash
# Vom Entwicklungs-PC aus (IP des Pi anpassen)
scp -r ./speedpilot/ros_backend pi@192.168.x.x:~/speedpilot/ros_backend
```

**Option C – USB-Stick:**
Projekt auf USB kopieren, am Pi einstecken und in ein lokales Verzeichnis kopieren.

### 3. Docker Image auf dem Pi bauen

```bash
cd ~/speedpilot/ros_backend/docker
bash build.sh
# Das Image wird für ARM64 (native Pi-Architektur) gebaut.
# Erster Build dauert ca. 15–30 Minuten.
```

### 4. Hardware verkabeln

#### Motor-Controller (H-Brücke)

| GPIO Pin (BCM) | Funktion          | Kabel      |
|---------------|-------------------|------------|
| 24            | Motor vorwärts    |            |
| 25            | Motor rückwärts   |            |
| 23            | Lenkservo (PWM)   |            |
| 20            | Modus-LED         |            |
| 16            | Status-Pin (Bridge)|           |

#### Ultraschallsensor (HC-SR04)

| GPIO Pin (BCM) | Funktion | Kabel |
|---------------|----------|-------|
| 11            | Trigger  | grün  |
| 9             | Echo     | blau  |

> **Spannung:** HC-SR04 arbeitet mit 5V, Echo-Ausgang muss mit Spannungsteiler (z.B. 1kΩ / 2kΩ) auf 3.3V gebracht werden, um den Pi nicht zu beschädigen.

#### LiDAR

LiDAR-Sensor per USB oder UART anschließen. Der scan-Topic muss vom LiDAR-Treiber auf `/scan` publishen (Standard für RPLiDAR, YDLIDAR, etc.).

### 5. Container starten

```bash
cd ~/speedpilot/ros_backend/docker
docker compose up -d
```

Der Container startet automatisch alle Abhängigkeiten und hält sich im Hintergrund am Laufen.

Logs ansehen:

```bash
docker logs -f speedpilot
```

### 6. ROS 2 Nodes starten

```bash
# In den Container wechseln
docker exec -it speedpilot bash

# Alle Nodes einzeln in separaten Terminals starten:
ros2 run ros2_bridge bridge_node          # WebSocket-Bridge (Port 9091)
ros2 run car_controller controller_node   # Motor + Servo
ros2 run ultrasonic_sensor ultrasonic_node # Ultraschall
ros2 run lidar_obstacle_avoidance obstacle_avoidance_node  # Autonome Vermeidung (optional)
```

Alternativ alle auf einmal im Hintergrund:

```bash
docker exec -d speedpilot bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /root/ros2_ws/install/setup.bash &&
  ros2 run ros2_bridge bridge_node &
  ros2 run car_controller controller_node &
  ros2 run ultrasonic_sensor ultrasonic_node
"
```

### 7. Flutter App verbinden

1. App auf dem Smartphone starten
2. Sicherstellen, dass Smartphone und Pi im selben WLAN sind
3. IP-Adresse des Pi ermitteln:

```bash
hostname -I
```

4. In der App: Gerät hinzufügen → IP des Pi eingeben → Verbinden
5. Die App verbindet sich auf `ws://<IP>:9091`

### 8. Autostart beim Booten (optional)

Um den Container automatisch beim Hochfahren des Pi zu starten:

```bash
# docker-compose.yml: restart-Policy setzen
# In ros_backend/docker/docker-compose.yml ergänzen:
#   restart: unless-stopped
```

Oder via systemd:

```bash
sudo nano /etc/systemd/system/speedpilot.service
```

```ini
[Unit]
Description=SpeedPilot ROS2 Backend
After=docker.service
Requires=docker.service

[Service]
WorkingDirectory=/home/pi/speedpilot/ros_backend/docker
ExecStart=docker compose up
ExecStop=docker compose down
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable speedpilot
sudo systemctl start speedpilot
```

---

## Architektur & Topics

| Topic                  | Typ                          | Publisher               | Subscriber              |
|------------------------|------------------------------|-------------------------|-------------------------|
| `/vehicle_command`     | `custom_msgs/VehicleCommand` | ros2_bridge             | car_controller          |
| `/vehicle_command`     | `custom_msgs/VehicleCommand` | lidar_obstacle_avoidance| car_controller          |
| `/scan`                | `sensor_msgs/LaserScan`      | LiDAR-Treiber           | ros2_bridge, lidar_obs. |
| `/map`                 | `nav_msgs/OccupancyGrid`     | SLAM-Node               | ros2_bridge             |
| `/pose`                | `geometry_msgs/PoseWithCovarianceStamped` | SLAM-Node | ros2_bridge            |
| `/ultrasonic/distance` | `std_msgs/Float32`           | ultrasonic_sensor       | car_controller          |

### WebSocket-Protokoll (Port 9091)

**App → Bridge (JSON):**

```json
{ "command": "move", "speed": 0.75, "angle": -0.3 }
{ "command": "stop", "speed": 0.0,  "angle": 0.0  }
```

**Bridge → App (JSON):**

```json
{ "type": "lidar",    "ranges": [...], "angle_min": ..., "angle_increment": ... }
{ "type": "map",      "width": 100, "height": 100, "data": [...] }
{ "type": "pose",     "x": 1.2, "y": 0.5, "theta": 0.3 }
```

---

## Verzeichnisstruktur

```
speedpilot/
├── frontend/           Flutter-App (Dart)
└── ros_backend/        ROS 2 Backend
    ├── docker/         Dockerfile, docker-compose, Build-Skripte
    └── ros2_ws/src/
        ├── ros2_bridge/              WebSocket ↔ ROS 2 Bridge
        ├── car_controller/           Motor- und Servosteuerung
        ├── lidar_obstacle_avoidance/ Autonome LiDAR-Hindernisvermeidung
        ├── ultrasonic_sensor/        HC-SR04 Abstandsmessung
        └── custom_msgs/              Eigene ROS 2 Message-Typen
```

---

## Lizenz

MIT — siehe [ros_backend/LICENSE](ros_backend/LICENSE)
