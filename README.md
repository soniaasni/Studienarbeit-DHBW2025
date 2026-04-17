# Speedpilot - Studienarbeit DHBW 2025

Ein autonomes Fahrzeugsystem bestehend aus einer Flutter-App (Frontend) und ROS 2 Backend (Fahrzeugsteuerung & Sensorik).

## 📁 Projektstruktur

```
Studienarbeit-DHBW2025/
├── speedpilot-main/              # Flutter Mobile App
│   ├── lib/                       # Dart Code
│   ├── assets/                    # Bilder & Daten
│   └── pubspec.yaml               # Flutter Dependencies
├── speedpilot_ros_backend-main/   # ROS 2 Backend
│   ├── ros2_ws/                   # ROS 2 Workspace
│   ├── docker/                    # Docker Setup
│   └── README.md                  # Backend Dokumentation
├── Studienarbeit/                 # LaTeX Dokumentation
└── README.md                      # (Diese Datei)
```

## 🚀 Schnelleinstieg

### Frontend (Flutter App)
```bash
cd speedpilot-main
flutter pub get
flutter run
```

### Backend (ROS 2)
```bash
cd speedpilot_ros_backend-main
docker compose up
```

## 📖 Dokumentation

- [Flutter App README](speedpilot-main/README.md)
- [ROS 2 Backend README](speedpilot_ros_backend-main/README.md)
- [Studienarbeit (LaTeX)](Studienarbeit/main.tex)

## 📝 Über das Projekt

Diese Studienarbeit wurde an der DHBW Ravensburg durchgeführt und behandelt die Entwicklung eines autonomen Fahrzeugsystems mit Sensor-Integration, Echtzeitsteuerung und Web-Kommunikation.