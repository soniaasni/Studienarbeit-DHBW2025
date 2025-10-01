import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:speedpilot/map_page/map_scrolling_page.dart';
import 'package:speedpilot/settings_page/settings_page.dart';
import './features/tachometer.dart';
import './features/gyroscope_data.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// A page displaying gyroscopic motion data, tachometer, and navigation controls.
class GyroscopePage extends StatelessWidget {
  final String imagePath;

  const GyroscopePage({Key? key, required this.imagePath}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    // Ensure Flutter bindings are initialized and force landscape orientation
    WidgetsFlutterBinding.ensureInitialized();
    SystemChrome.setPreferredOrientations(
        [DeviceOrientation.landscapeLeft, DeviceOrientation.landscapeRight]);

    return Scaffold(
      backgroundColor: const Color.fromARGB(250, 25, 25, 25),
      body: Stack(
        children: <Widget>[
          // Display the tachometer at the bottom center
          Align(
            alignment: Alignment.bottomCenter,
            child: Tachometer(),
          ),

          // (Optional) Lidar visualization (commented out in current version)
          /*
          Align(
            alignment: Alignment.topCenter,
            child: Container(
              margin: const EdgeInsets.only(top: 10),
              height: 225,
              width: 225,
              child: LidarScreen(),
            ),
          ),
          */

          // Settings button at top right
          Positioned(
            top: 20,
            right: 50,
            child: IconButton(
              icon: Icon(Icons.settings, color: Colors.white),
              onPressed: () async {
                // Re-lock orientation to landscape before navigating
                WidgetsFlutterBinding.ensureInitialized();
                SystemChrome.setPreferredOrientations([
                  DeviceOrientation.landscapeLeft,
                  DeviceOrientation.landscapeRight
                ]);
                Navigator.pushReplacement(
                  context,
                  MaterialPageRoute(builder: (context) => Settings()),
                );
              },
            ),
          ),

          // Back button at top left to return to MapScrolling page
          Positioned(
            top: 20,
            left: 50,
            child: IconButton(
              icon: Icon(Icons.navigate_before, color: Colors.white),
              onPressed: () async {
                final prefs = await SharedPreferences.getInstance();
                await prefs.setBool('gyro', false); // Reset gyro state so joystick stearing appears every time the user enters the steering page
                SystemChrome.setPreferredOrientations([
                  DeviceOrientation.portraitUp,
                  DeviceOrientation.portraitDown
                ]);
                Navigator.push(
                  context,
                  MaterialPageRoute(builder: (context) => MapScrolling()),
                );
              },
            ),
          ),

          // Gyroscope data visualization centered at the top
          Align(
            alignment: Alignment.topCenter,
            child: Container(
              height: 200,
              width: 200,
              child: getGyroscope(), // Displays pitch, roll, and yaw
            ),
          ),
        ],
      ),
    );
  }
}
