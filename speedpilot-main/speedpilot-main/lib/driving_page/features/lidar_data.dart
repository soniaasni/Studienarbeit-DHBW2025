import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart' show rootBundle;
import './occupancy_grid.dart';

/// Widget that displays LIDAR data in the form of an occupancy grid.
/// The data is loaded from a JSON file stored in the app's assets.
class LidarScreen extends StatefulWidget {
  const LidarScreen({super.key});

  @override
  State<LidarScreen> createState() => _LidarScreenState();
}

class _LidarScreenState extends State<LidarScreen> {
  // OccupancyGrid object to hold parsed LIDAR data
  OccupancyGrid? grid;

  @override
  void initState() {
    super.initState();
    // Load LIDAR data on widget initialization
    loadLidarData();
  }

  /// Loads LIDAR occupancy grid data from a JSON file in assets
  Future<void> loadLidarData() async {
    try {
      // Load JSON string from the asset
      String jsonStr = await rootBundle
          .loadString('assets/lidar_data/occupancy_grid_example.json');

      // Decode JSON
      Map<String, dynamic> jsonMap = json.decode(jsonStr);

      // Convert to OccupancyGrid object
      setState(() {
        grid = OccupancyGrid.fromJson(jsonMap);
      });
    } catch (e, stacktrace) {
      // Log any error that occurs during loading or parsing
      print("Error loading JSON file: $e");
      print(stacktrace);
    }
  }

  @override
  Widget build(BuildContext context) {
    // Show loading spinner while data is being loaded
    if (grid == null) {
      return const Scaffold(
        body: Center(child: CircularProgressIndicator()),
      );
    }

    // Once the grid is loaded, display it using a GridView
    return Scaffold(
      body: GridView.builder(
        itemCount: grid!.data.length, // Total number of cells
        gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
          crossAxisCount: grid!.mapMetaData.width, // Width in number of cells
        ),
        itemBuilder: (context, index) {
          int value = grid!.data[index];
          Color color;

          // Assign colors based on occupancy value
          if (value == -1) {
            color = const Color.fromARGB(255, 0, 0, 0); // Unknown cell
          } else if (value == 0) {
            color = const Color.fromARGB(255, 0, 0, 0); // Free space
          } else {
            color = Colors.white; // Occupied cell
          }

          return Container(
            color: color,
            margin: const EdgeInsets.all(0.1), // Slight spacing between cells
          );
        },
      ),
    );
  }
}
