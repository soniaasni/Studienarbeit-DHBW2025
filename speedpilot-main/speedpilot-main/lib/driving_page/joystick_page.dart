import 'package:flutter/material.dart';
import 'package:speedpilot/map_page/map_scrolling_page.dart';
import './features/tachometer.dart';
import 'package:flutter/services.dart';
import './features/steering_joystick.dart';
import './features/gas_joystick.dart';
import 'package:speedpilot/settings_page/settings_page.dart';
import '../driving_page/features/lidar_data.dart';
import 'package:shared_preferences/shared_preferences.dart';

// Main widget for the joystick control page
class JoystickPage extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    // Ensure bindings are initialized before setting orientation
    WidgetsFlutterBinding.ensureInitialized();

    // Lock device orientation to landscape mode
    SystemChrome.setPreferredOrientations(
        [DeviceOrientation.landscapeLeft, DeviceOrientation.landscapeRight]);

    return Scaffold(
      backgroundColor: const Color.fromARGB(250, 25, 25, 25),
      body: Stack(
        children: <Widget>[
          // Tachometer aligned at the bottom center
          Align(
            alignment: Alignment.bottomCenter,
            child: Tachometer(),
          ),

          // Uncomment below if Lidar view should be displayed at top center
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
                // Re-set orientation to landscape in case it's changed
                WidgetsFlutterBinding.ensureInitialized();
                SystemChrome.setPreferredOrientations([
                  DeviceOrientation.landscapeLeft,
                  DeviceOrientation.landscapeRight
                ]);
                // Navigate to settings page
                Navigator.pushReplacement(
                  context,
                  MaterialPageRoute(builder: (context) => Settings()),
                );
              },
            ),
          ),

          // Steering joystick at bottom right
          Positioned(bottom: 7, right: 40, child: SteeringJoystickPage()),

          // Gas joystick at bottom left
          Positioned(bottom: 10, left: 40, child: GasJoystickPage()),

          // Back navigation button at top left
          Positioned(
            top: 20,
            left: 50,
            child: IconButton(
              icon: Icon(Icons.navigate_before, color: Colors.white),
              onPressed: () async {
                // Disable gyro preference before navigating back
                final prefs = await SharedPreferences.getInstance();
                await prefs.setBool('gyro', false);

                // Set orientation back to portrait mode
                SystemChrome.setPreferredOrientations([
                  DeviceOrientation.portraitUp,
                  DeviceOrientation.portraitDown
                ]);

                // Navigate back to the map scrolling page
                Navigator.push(
                  context,
                  MaterialPageRoute(builder: (context) => MapScrolling()),
                );
              },
            ),
          ),
        ],
      ),
    );
  }
}
