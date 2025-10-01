import 'package:flutter/material.dart';
import './starting_page/starting_page.dart';

// Entry point of the application
void main() {
  runApp(MyApp());
}

// Root widget of the app
class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      // Removes the debug banner from the top right corner
      debugShowCheckedModeBanner: false,

      // Sets the first screen of the app
      home: StartingPage(),
    );
  }
}
