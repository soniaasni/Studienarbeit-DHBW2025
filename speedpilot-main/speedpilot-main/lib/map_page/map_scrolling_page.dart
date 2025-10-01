import 'package:flutter/material.dart';
import './features/map_carousel.dart';
import 'package:flutter/services.dart';
import 'package:speedpilot/starting_page/starting_page.dart';
import 'package:speedpilot/services/WebSocketManager.dart';

class MapScrolling extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    // Ensure widget binding is initialized and lock to portrait mode
    WidgetsFlutterBinding.ensureInitialized();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);

    return Scaffold(
      backgroundColor: const Color.fromARGB(250, 25, 25, 25),
      body: Stack(
        children: <Widget>[
          // Main carousel content
          CustomCarousel(),
          
          // Back button at top left
          Positioned(
            top: 50,
            left: 20,
            child: IconButton(
              icon: Icon(Icons.navigate_before, color: Colors.white),
              onPressed: () {
                // Close WebSocket connection and return to starting page
                WebSocketManager().closeConnection();
                Navigator.push(
                  context,
                  MaterialPageRoute(builder: (context) => StartingPage()),
                );
              },
            ),
          ),
        ],
      ),
    );
  }
}
