import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:speedpilot/services/WebSocketManager.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// This widget represents a gas (throttle) control using a vertical joystick.
class GasJoystickPage extends StatefulWidget {
  @override
  _GasJoystickPageState createState() => _GasJoystickPageState();
}

class _GasJoystickPageState extends State<GasJoystickPage> {
  /// Saves the current speed value to local storage (shared preferences).
  Future<void> _getSpeedData(double value) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setDouble('speed', value);
  }

  // Current speed value, updated from joystick input.
  double speedy = 0.0;

  @override
  Widget build(BuildContext context) {
    return Container(
      child: Joystick(
        mode: JoystickMode.vertical, // Vertical-only movement
        base: JoystickBase(
          mode: JoystickMode.vertical,
          decoration: JoystickBaseDecoration(
            drawOuterCircle: false,
            color: Color.fromARGB(250, 25, 25, 25), // Base color (dark)
            drawInnerCircle: false,
            drawMiddleCircle: false,
          ),
        ),
        stick: JoystickStick(
          decoration: JoystickStickDecoration(
            color: Colors.red, // The stick color
          ),
        ),
        listener: (StickDragDetails details) {
          // Send speed data via WebSocket (inverted Y value)
          WebSocketManager().updateSpeed(-details.y);

          // Convert and store speed locally (scaled by 8) to simulate the velocity on the Tachometer based on the movement of the joystick
          speedy = details.y * -1 * 8;
          _getSpeedData(speedy);
        },
      ),
    );
  }
}
