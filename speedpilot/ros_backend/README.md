# SpeedPilot – ROS 2 Backend

ROS 2 Jazzy Backend für das SpeedPilot-Fahrzeugsteuerungssystem. Läuft als Docker-Container auf einem Raspberry Pi 4 und steuert Motor, Lenkservo sowie optionale Sensorik.

## Packages

| Package                    | Beschreibung                                                                 |
|---------------------------|------------------------------------------------------------------------------|
| `ros2_bridge`             | WebSocket-Server (Port 9091), Brücke zwischen Flutter-App und ROS 2          |
| `car_controller`          | PWM-Steuerung für Motor (GPIO 24/25) und Servo (GPIO 23) via RPi.GPIO        |
| `lidar_obstacle_avoidance`| Autonome Hindernisvermeidung basierend auf LiDAR `/scan`-Topic               |
| `ultrasonic_sensor`       | HC-SR04 Abstandsmessung, publiziert auf `/ultrasonic/distance` (Float32)     |
| `custom_msgs`             | Message-Definition `VehicleCommand` (`command`, `speed`, `angle`)            |

## GPIO-Belegung

```
BCM Pin 24 → Motor vorwärts  (PWM 50 Hz)
BCM Pin 25 → Motor rückwärts (PWM 50 Hz)
BCM Pin 23 → Lenkservo       (PWM 50 Hz, 5–10 % Duty Cycle)
BCM Pin 20 → Modus-LED       (HIGH = aktiv)
BCM Pin 16 → Bridge Status   (HIGH beim Start)
BCM Pin 11 → Ultraschall Trigger (HC-SR04)
BCM Pin  9 → Ultraschall Echo    (HC-SR04, 3.3V!)
```

## Docker

### Image bauen

```bash
cd docker
bash build.sh
```

### Container starten

```bash
docker compose up -d
```

### In den Container wechseln

```bash
docker exec -it speedpilot bash
```

### Workspace neu bauen (im Container)

```bash
cd /root/ros2_ws
colcon build --packages-select ros2_bridge car_controller lidar_obstacle_avoidance ultrasonic_sensor custom_msgs
source install/setup.bash
```

## Nodes starten

```bash
# WebSocket-Bridge (muss zuerst laufen)
ros2 run ros2_bridge bridge_node

# Motor + Servo
ros2 run car_controller controller_node

# Ultraschall
ros2 run ultrasonic_sensor ultrasonic_node

# Autonome Hindernisvermeidung (optional, übernimmt vehicle_command)
ros2 run lidar_obstacle_avoidance obstacle_avoidance_node
```

## Umgebungsvariablen

| Variable         | Standard | Beschreibung                                      |
|-----------------|---------|---------------------------------------------------|
| `ROS_DOMAIN_ID` | `0`     | Wert 0–232, für Multi-Roboter-Trennung anpassen   |

`ROS_DOMAIN_ID` wird beim ersten Start in `/root/shared/ros2/ros_domain_id.txt` gespeichert.

## Verzeichnisstruktur

```
ros_backend/
├── install_native.sh       Direktinstallation auf dem Pi (ohne Docker)
├── docker/
│   ├── Dockerfile          Multi-Stage Build (Builder + Runtime)
│   ├── docker-compose.yml  Container-Konfiguration mit GPIO-Zugriff
│   ├── build.sh            Image bauen (speedpilot:latest)
│   ├── entrypoint.sh       Container-Startskript (Build + ROS-Setup)
│   ├── workspace.sh        Workspace-Initialisierung im Container
│   └── bash_aliases.txt    Aliases für den Container
└── ros2_ws/
    └── src/
        ├── ros2_bridge/
        ├── car_controller/
        ├── lidar_obstacle_avoidance/
        ├── ultrasonic_sensor/
        └── custom_msgs/
```

## Hinweise

- Der Container benötigt `--privileged` für GPIO-Zugriff (`/dev/gpiomem`, `/dev/mem`).
- `network_mode: host` ist erforderlich, damit Port 9091 direkt auf dem Pi erreichbar ist.
- Beim ersten Start baut der Entrypoint die gesamte Workspace mit `colcon build`.
- `install_native.sh` kann direkt auf dem Pi ausgeführt werden, um Abhängigkeiten ohne Docker zu installieren.
