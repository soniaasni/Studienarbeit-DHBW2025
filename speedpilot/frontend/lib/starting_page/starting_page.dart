import 'package:flutter/material.dart';
import 'features/device_list.dart';
import 'package:flutter/services.dart';
import 'package:speedpilot/settings_page/settings_page.dart';
import 'package:speedpilot/services/WebSocketManager.dart';

// Entry point of the app after startup — displays list of available devices
class StartingPage extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    // Lock orientation to portrait mode
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
    ]);

    return Scaffold(
      appBar: AppBar(
        backgroundColor: const Color.fromARGB(250, 25, 25, 25),
        
        // Logout icon button (currently only closes WebSocket)
        leading: IconButton(
          icon: Icon(Icons.logout, color: Colors.white),
          onPressed: () {
            WebSocketManager().closeConnection(); // Close WebSocket when logging out
          },
        ),
      ),

      // Body shows the device selection list
      body: Devices(),
    );
  }
}
