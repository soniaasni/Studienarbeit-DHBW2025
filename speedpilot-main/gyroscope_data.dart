import 'dart:async';
import 'package:flutter/material.dart';
import 'package:motion_sensors/motion_sensors.dart';
import 'package:speedpilot/services/WebSocketManager.dart';

/// Widget that listens to gyroscope/orientation data and sends control values
/// (speed and steering angle) via WebSocket.
class getGyroscope extends StatefulWidget {
  @override
  State<getGyroscope> createState() => _getGyroscope();
}

class _getGyroscope extends State<getGyroscope> {
  // Orientation values
  double _motionPitch = 0.0;
  double _motionRoll = 0.0;
  double _motionYaw = 0.0;

  // Subscription to orientation sensor data stream
  late final StreamSubscription<OrientationEvent> _orientationSubscription;

  @override
  void initState() {
    super.initState();

    // Listen to device orientation (pitch, roll, yaw)
    _orientationSubscription =
        motionSensors.orientation.listen((OrientationEvent event) {
      // Clamp pitch and roll to a safe range [-1.0, 1.0]
      double pitch = event.pitch.clamp(-1.0, 1.0);
      double roll = event.roll.clamp(-1.0, 1.0);

      setState(() {
        _motionPitch = pitch;
        _motionRoll = roll;
        _motionYaw = event.yaw;
      });

      // Send movement commands via WebSocket
      WebSocketManager().updateSpeed(roll);   // Roll = forward/backward motion
      WebSocketManager().updateAngle(pitch);  // Pitch = steering (left/right)
    });
  }

  @override
  void dispose() {
    // Cancel the sensor data subscription when the widget is removed
    _orientationSubscription.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 80,
      color: const Color.fromARGB(250, 25, 25, 25), 
      // Displays the Values on the application screen
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text('Motion Data:', style: TextStyle(color: Colors.white)),
          Text('Pitch: $_motionPitch', style: TextStyle(color: Colors.white)),
          Text('Roll: $_motionRoll', style: TextStyle(color: Colors.white)),
          Text('Yaw: $_motionYaw', style: TextStyle(color: Colors.white)),
        ],
      ),
    );
  }
}
