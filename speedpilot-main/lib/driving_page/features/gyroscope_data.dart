import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'package:speedpilot/services/WebSocketManager.dart';

/// ------------------------------------------------------------
/// getGyroscope Widget
/// ------------------------------------------------------------
/// Zweck:
/// - Liest Gyroskop-Daten (Drehgeschwindigkeit in rad/s)
/// - Glättet die Werte (damit Steuerung nicht zittert)
/// - Mappt Gyro-Werte -> Steuerwerte (speed / steering)
/// - Schickt diese Steuerwerte per WebSocket an dein Fahrzeug
/// - Zeigt Debug-Werte in der UI an
///
/// WICHTIG:
/// - Accelerometer -> kann "Neigung" (Pitch/Roll) schätzen, weil es Gravitation sieht
/// - Gyroskop -> misst Drehgeschwindigkeit (nur Ausschlag beim Bewegen, nicht beim Halten)
class getGyroscope extends StatefulWidget {
  @override
  State<getGyroscope> createState() => _getGyroscope();
}

class _getGyroscope extends State<getGyroscope> {
  // ============================================================
  // 1) UI / Debug-Anzeige (nur zum Anzeigen in der App)
  // ============================================================
  // Diese Variablen haben keinen direkten Einfluss auf Steuerung,
  // außer du benutzt sie später. Sie sind primär "was wird angezeigt".
  double _motionPitch = 0.0; // Debug-Wert (aktueller gx)
  double _motionRoll  = 0.0; // Debug-Wert (aktueller gy)
  double _motionYaw   = 0.0; // Debug-Wert (aktueller gz)

  // ============================================================
  // 2) Stream Subscriptions (Sensor-Listener)
  // ============================================================
  // Existieren, damit du die Sensor-Streams im dispose() sauber beenden kannst.
  // Sonst laufen sie weiter -> Memory Leak / Crash / doppelte Events.
  StreamSubscription<AccelerometerEvent>? _accelSub;

  // ============================================================
  // 3) GLÄTTUNG (Low-Pass Filter)
  // ============================================================
  // Warum existiert das?
  // Gyro-Daten sind oft "rauschig" -> kleine Zitterbewegungen.
  // Der Filter macht die Steuerung ruhiger.
  //
  // >>> HIER kannst du die Reaktionsfreudigkeit anpassen <<<
  //
  // alpha nahe 1.0  -> sehr glatt, aber träge (langsamer reagiert)
  // alpha nahe 0.0  -> kaum Glättung, aber nervöser (zittert mehr)
  static const double _alpha = 0.85; // <<< WERT ÄNDERN FÜR "SMOOTHNESS"
  double _smoothedSteer = 0.0; // gefiltert (für steering)
  double _smoothedSpeed = 0.0; // gefiltert (für speed)

  // ============================================================
  // 4) MAPPING / SCALING (Gyro -> Fahrzeug)
  // ============================================================
  // Warum existiert das?
  // Gyro liefert rad/s, aber dein Fahrzeug erwartet wahrscheinlich Werte wie:
  // -1..1 oder 0..1 oder -100..100 etc.
  //
  // >>> HIER passt du die "Empfindlichkeit" an <<<
  //
  // Beispiel:
  // - Wenn Steering zu stark reagiert -> kleiner machen
  // - Wenn Steering zu schwach reagiert -> größer machen
  //
  // steeringGain: wie stark Lenken reagiert
  // speedGain:    wie stark Geschwindigkeit reagiert
  //
  // STARTWERTE (anpassen!)
  static const double steeringGain = 1.0; // <<< WERT ÄNDERN: Lenksensitivität
  static const double speedGain = 1.0;    // <<< WERT ÄNDERN: Speed-Sensitivität

  // ============================================================
  // 5) DEADZONE (kleine Bewegungen ignorieren)
  // ============================================================
  // Warum existiert das?
  // Selbst wenn du das Handy "still hältst", liefert Gyro kleine Werte.
  // Deadzone sorgt: wenn |wert| < deadzone -> 0.
  //
  // >>> HIER änderst du, wie “still” wirklich still ist <<<
  static const double deadzone = 0.03; // <<< WERT ÄNDERN: Zittern wegfiltern

  // ============================================================
  // 6) OUTPUT CLAMP (Sicherheitsgrenze)
  // ============================================================
  // Warum existiert das?
  // Damit du garantiert im erwarteten Bereich bleibst (z.B. -1..1),
  // egal was Sensor liefert.
  //
  // >>> ÄNDERN nur, wenn Fahrzeug andere Range erwartet! <<<
  static const double outMin = -1.0;
  static const double outMax = 1.0;

  // ============================================================
  // 7) OPTIONAL: Achsen tauschen / invertieren
  // ============================================================
  // Warum existiert das?
  // Je nachdem wie du das Handy hältst, ist "links/rechts" evtl. auf der falschen Achse
  // oder invertiert (links lenkt rechts).
  //
  // >>> HIER kannst du schnell fixen, ohne alles umzuschreiben <<<
  static const bool invertSteering = false; // <<< true, wenn links/rechts vertauscht
  static const bool invertSpeed = false;    // <<< true, wenn vor/zurück vertauscht

  @override
  void initState() {
    super.initState();

     // ============================================================
    // ACCELEROMETER LISTENER (KERN der Neigungssteuerung)
    // ============================================================
    // accelerometerEvents liefert Beschleunigung in m/s² entlang x,y,z.
    // Wenn Handy ruhig ist, misst es hauptsächlich Gravitation.
    //
    // Daraus kann man Pitch/Roll berechnen:
    // - Pitch: vor/zurück kippen
    // - Roll: links/rechts kippen
    _accelSub = accelerometerEvents.listen(
      (AccelerometerEvent a) {
        // --------------------------------------------------------
        // 1) Pitch/Roll aus Accelerometer berechnen (in rad)
        // --------------------------------------------------------
        // pitch: vor/zurück
        final pitch = atan2(-a.x, sqrt(a.y * a.y + a.z * a.z));

        // roll: links/rechts
        final roll = atan2(a.y, a.z);

        // --------------------------------------------------------
        // 2) Mapping: Welche Neigung steuert was?
        // --------------------------------------------------------
        // Das ist der wichtigste Teil:
        // - Speed soll von "nach vorne kippen" abhängen -> pitch
        // - Steering soll von "links/rechts kippen" abhängen -> roll
        double rawSpeed = pitch; // <<< HIER ändern, wenn du andere Achse willst
        double rawSteer = roll;  // <<< HIER ändern, wenn du andere Achse willst

        // Optional invertieren (wenn Richtung falsch herum)
        if (invertSpeed) rawSpeed = -rawSpeed;
        if (invertSteering) rawSteer = -rawSteer;

        // --------------------------------------------------------
        // 3) Deadzone (Zittern & kleine Bewegungen ignorieren)
        // --------------------------------------------------------
        rawSpeed = _applyDeadzone(rawSpeed, deadzone);
        rawSteer = _applyDeadzone(rawSteer, deadzone);

        // --------------------------------------------------------
        // 4) Gain (Empfindlichkeit)
        // --------------------------------------------------------
        rawSpeed *= speedGain;
        rawSteer *= steeringGain;

        // --------------------------------------------------------
        // 5) Glätten (ruhigere Steuerung)
        // --------------------------------------------------------
        _smoothedSpeed = _alpha * _smoothedSpeed + (1 - _alpha) * rawSpeed;
        _smoothedSteer = _alpha * _smoothedSteer + (1 - _alpha) * rawSteer;

        // --------------------------------------------------------
        // 6) Clamp auf Output-Range
        // --------------------------------------------------------
        final sendSpeed = _smoothedSpeed.clamp(outMin, outMax);
        final sendAngle = _smoothedSteer.clamp(outMin, outMax);

        // --------------------------------------------------------
        // 7) Werte senden
        // --------------------------------------------------------
        // Wenn dein Fahrzeug statt -1..1 z.B. 0..1 erwartet:
        //   double mapped = (value + 1) / 2;  // -1..1 -> 0..1
        // Dann hier vorher umrechnen.
        WebSocketManager().updateSpeed(sendSpeed);
        WebSocketManager().updateAngle(sendAngle);

        // --------------------------------------------------------
        // 8) UI / Debug aktualisieren
        // --------------------------------------------------------
        // Hier zeigen wir die berechneten Winkel (rad) an.
        if (!mounted) return;
        setState(() {
          _motionPitch = pitch;
          _motionRoll = roll;
          _motionYaw = 0.0; // accel hat kein yaw
        });
      },
      onError: (e) {
        debugPrint('Accelerometer error: $e');
      },
    );
  }

  // ------------------------------------------------------------
  // Helper: Deadzone
  // ------------------------------------------------------------
  // - Wenn |v| < dz, dann 0
  // - Sonst bleibt v
  // Das macht die Steuerung im "Stillstand" ruhiger.
  double _applyDeadzone(double v, double dz) {
    if (v.abs() < dz) return 0.0;
    return v;
  }

  @override
  void dispose() {
    // Streams beenden, sonst laufen sie weiter
    _accelSub?.cancel();
    super.dispose();
  }

  // ============================================================
  // UI (alte Anzeige wieder)
  // ============================================================
  @override
  Widget build(BuildContext context) {
    return Container(
      height: 80,
      color: const Color.fromARGB(250, 25, 25, 25),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          const Text('Motion Data:', style: TextStyle(color: Colors.white)),
          // Anzeige der Winkel in rad (für Menschen oft schwer)
          // Tipp: rad -> Grad: rad * 180 / pi
          Text('Pitch: ${_motionPitch.toStringAsFixed(4)}',
              style: const TextStyle(color: Colors.white)),
          Text('Roll:  ${_motionRoll.toStringAsFixed(4)}',
              style: const TextStyle(color: Colors.white)),
          Text('Yaw:   ${_motionYaw.toStringAsFixed(4)}',
              style: const TextStyle(color: Colors.white)),
        ],
      ),
    );
  }
}