import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:speedpilot/driving_page/gyroscope_page.dart';
import 'package:speedpilot/driving_page/joystick_page.dart';
import './features/settings_list.dart';
import 'package:shared_preferences/shared_preferences.dart';

// Dummy alternative page (not currently used, just for placeholder/testing)
class AlternativePage extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Text(
          "Alternative Seite",
          style: TextStyle(color: Colors.white),
        ),
      ),
      backgroundColor: Colors.black,
    );
  }
}

// Main Settings page
class Settings extends StatefulWidget {
  @override
  _SettingsState createState() => _SettingsState();
}

class _SettingsState extends State<Settings> {
  bool _switchState = false; // Tracks the state of the gyro setting

  get images => null;

  @override
  void initState() {
    super.initState();

    // Ensure binding and lock orientation to landscape
    WidgetsFlutterBinding.ensureInitialized();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);

    // Load stored switch state from SharedPreferences
    _loadInitialState();
  }

  // Load the current value of 'gyro' from SharedPreferences
  Future<void> _loadInitialState() async {
    final prefs = await SharedPreferences.getInstance();
    setState(() {
      _switchState = prefs.getBool('gyro') ?? false;
    });
  }

  // Handle back button: navigate to appropriate page based on gyro setting
  void _onBeforePressed() {
    if (!_switchState) {
      // If gyro is off, go to joystick control
      Navigator.pushReplacement(
        context,
        MaterialPageRoute(builder: (_) => JoystickPage()),
      );
    } else {
      // If gyro is on, go to gyroscope page
      Navigator.pushReplacement(
        context,
        MaterialPageRoute(
          builder: (_) => GyroscopePage(
            imagePath: '',
          ),
        ),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Color.fromARGB(250, 25, 25, 25),
      body: Stack(
        children: [
          // Settings list (with switch)
          Align(
            alignment: Alignment.topCenter,
            child: Container(
              margin: EdgeInsets.only(top: 50),
              child: SettingsList(
                onSwitchChanged: (val) {
                  _switchState = val; // Update local state (not triggering rebuild)
                },
              ),
            ),
          ),
          // Back button (top left)
          Positioned(
            top: 20,
            left: 50,
            child: IconButton(
              icon: Icon(Icons.navigate_before, color: Colors.white),
              onPressed: _onBeforePressed,
            ),
          ),
        ],
      ),
    );
  }
}
