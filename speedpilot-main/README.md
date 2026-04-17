# 🛰️ Remote Vehicle Control App

Eine Flutter-basierte mobile Applikation zur Fernsteuerung eines Fahrzeugs, ausgestattet mit Live-Sensorintegration, Kartenvisualisierung und Joystick-/Gyroskopsteuerung.

---

## 📁 Projektstruktur

```
lib/
├── main.dart                      # Einstiegspunkt der App
├── starting_page/                # Startbildschirm und Geräteverbindung
│   ├── starting_page.dart
│   └── features/
│       ├── device_list.dart      # Anzeige von vorhandenen Autos mir IP-Adresse
│       └── user_dialogue.dart    # Nutzerabfragen & Dialoge
├── driving_page/                 # Steuerungsseite für das Fahrzeug
│   ├── joystick_page.dart        # Joystick-basierte Steuerung
│   ├── gyroscope_page.dart       # Steuerung über Gyroskop
│   └── features/
│       ├── gas_joystick.dart
│       ├── steering_joystick.dart
│       ├── lidar_data.dart
│       ├── occupancy_grid.dart
│       ├── tachometer.dart
│       └── gyroscope_data.dart
├── map_page/                     # Kartenseite mit Scroll-/Zoom-Funktion
│   ├── map_scrolling_page.dart
│   └── features/
│       └── map_carousel.dart     # Karussell mehrerer Maps
├── models/
│   └── card_list.dart            # Datenmodell zum Hinzufügen neuer Karten zur Liste
```

---

## ⚙️ Installation

### Voraussetzungen

- [Flutter SDK](https://flutter.dev/docs/get-started/install)
- Android Studio / Xcode (für Emulatoren oder echte Geräte)
- Git (optional)

### Schritte

```bash
# 1. Projekt klonen
git clone <repo-url>
cd <projektverzeichnis>

# 2. Flutter-Abhängigkeiten installieren
flutter pub get

# 3. Plattform-spezifische Abhängigkeiten vorbereiten
flutter clean
flutter pub get
```

---

## 🚀 Ausführung

### Android Emulator

```bash
flutter emulators --launch <emulator_id>
flutter run
```

> Alternativ im Android Studio über den Button **"Run"** starten

---

### iOS Simulator

```bash
open -a Simulator
flutter run
```

> Hinweis: Für iOS wird ein Mac mit Xcode benötigt.

---

### Echtes Android-Gerät

1. USB-Debugging aktivieren
2. Gerät über USB verbinden
3. Mit `flutter devices` sicherstellen, dass es erkannt wird
4. Dann:

```bash
flutter run
```

---

### Echtes iOS-Gerät

1. Apple Developer-Konto notwendig
2. Zertifikate in Xcode konfigurieren
3. Gerät anschließen und autorisieren
4. Ausführung via Xcode oder:

```bash
flutter run
```

---

## 🧭 Nutzung

1. **Startseite**  
   → Geräte werden gescannt und verbunden

2. **Fahrsteuerung**  
   → Joystick: Gaspedal, Lenkung, Anzeige von Lidar und Gyroskopdaten  
   → Alternativsteuerung: Gyroskop-basierte Eingabe

3. **Kartenansicht**  
   → Anzeige von Kartenmaterial, Karussell-Ansicht und Zoom/Scroll-Funktion

---

## 👨‍💻 Mitwirkende

- **Tobias Schilling**
- **Max Domitrovic**

---

## 📄 Lizenz

Dieses Projekt steht unter der **MIT-Lizenz**:

```
MIT License

Copyright (c) 2025 Tobias Schilling, Max Domitrovic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction...
```

---

## ❓ FAQ

**Q: Muss ich native SDKs installieren?**  
A: Ja, Flutter benötigt Android SDK und Xcode für native Builds.

**Q: Kann ich das Projekt im Web laufen lassen?**  
A: Aktuell ist es für Android und iOS optimiert. Web-Support wäre möglich, ist aber nicht enthalten.
