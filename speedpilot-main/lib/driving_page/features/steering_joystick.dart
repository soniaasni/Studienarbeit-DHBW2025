import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:speedpilot/services/WebSocketManager.dart';

/// A widget that provides a horizontal joystick for steering control.
/// Sends angle data to a WebSocket server in real-time.
class SteeringJoystickPage extends StatefulWidget {
  const SteeringJoystickPage({super.key});

  @override
  _JoystickPageState createState() => _JoystickPageState();
}

class _JoystickPageState extends State<SteeringJoystickPage> {
  @override
  Widget build(BuildContext context) {
    return Joystick(
      // The joystick is limited to horizontal movement only
      mode: JoystickMode.horizontal,

      // Customize the joystick base (no outer/inner/middle circles)
      base: JoystickBase(
        mode: JoystickMode.horizontal,
        decoration: JoystickBaseDecoration(
          drawOuterCircle: false,
          color: Color.fromARGB(250, 25, 25, 25), // Dark background
          drawInnerCircle: false,
          drawMiddleCircle: false,
        ),
      ),

      // Customize the joystick stick
      stick: JoystickStick(
        decoration: JoystickStickDecoration(
          color: Colors.red, // Red stick for visibility
        ),
      ),

      // When the joystick is moved, send the angle value (x-axis) over WebSocket
      listener: (StickDragDetails details) {
        WebSocketManager().updateAngle(details.x);
      },
    );
  }
}
