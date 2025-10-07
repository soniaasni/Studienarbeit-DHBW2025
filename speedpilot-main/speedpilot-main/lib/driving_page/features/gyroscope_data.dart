import 'dart:async';
import 'package:flutter/material.dart';
// import 'package:motion_sensors/motion_sensors.dart';
import 'package:sensors_plus/sensors_plus.dart';
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

 // late final StreamSubscription<OrientationEvent> _orientationSubscription;

@override
void initState() {
  super.initState();

  // Temporär deaktiviert, weil motion_sensors nicht verwendet werden kann!
  // _orientationSubscription =
  //     motionSensors.orientation.listen((OrientationEvent event) {
  //   double pitch = event.pitch.clamp(-1.0, 1.0);
  //   double roll = event.roll.clamp(-1.0, 1.0);

  //   setState(() {
  //     _motionPitch = pitch;
  //     _motionRoll = roll;
  //     _motionYaw = event.yaw;
  //   });

  //   WebSocketManager().updateSpeed(roll);   // Roll = forward/backward motion
  //   WebSocketManager().updateAngle(pitch);  // Pitch = steering (left/right)
  // });

  // Optional: Dummywerte setzen, damit UI nicht abstürzt
  setState(() {
    _motionPitch = 0.0;
    _motionRoll = 0.0;
    _motionYaw = 0.0;
  });
}

// Vergiss nicht, dispose ggf. auch anzupassen (Subscription existiert nicht mehr):
// @override
// void dispose() {
//   _orientationSubscription.cancel();
//   super.dispose();
// }

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
