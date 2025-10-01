import 'dart:async';
import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';

// Singleton class to manage WebSocket connection and driving commands
class WebSocketManager {
  // Singleton instance
  static final WebSocketManager _instance = WebSocketManager._internal();

  // Factory constructor returns the singleton instance
  factory WebSocketManager() {
    return _instance;
  }

  // Private constructor
  WebSocketManager._internal();

  WebSocketChannel? _channel;
  StreamSubscription? _subscription;

  // Internal state for current speed and steering angle
  double _currentSpeed = 0.0;
  double _currentAngle = 0.0;

  // Optional handler for incoming lidar data
  void Function(Map<String, dynamic>)? onLidarDataReceived;

  // Connect to the WebSocket server with a given URL
  Future<void> connect(String url) async {
    final uri = Uri.parse(url);
    _channel = WebSocketChannel.connect(uri);
  }

  // Send raw string message via WebSocket
  void sendMessage(String message) {
    _channel?.sink.add(message);
    print("Message sent: $message");
  }

  // Send a JSON driving command containing current speed and angle
  void _sendDrivingCommand() {
    final Map<String, dynamic> jsonData = {
      "command": "move",
      "speed": _currentSpeed,
      "angle": _currentAngle
    };
    final String jsonString = jsonEncode(jsonData);
    sendMessage(jsonString);
  }

  // Update speed and send new driving command
  void updateSpeed(double speed) {
    _currentSpeed = speed;
    _sendDrivingCommand();
  }

  // Update angle and send new driving command
  void updateAngle(double angle) {
    _currentAngle = angle;
    _sendDrivingCommand();
  }

  // Listen for incoming WebSocket messages
  void receiveMessage(void Function(dynamic message) onMessage) {
    _subscription = _channel?.stream.listen(
      (message) {
        try {
          final decoded = jsonDecode(message);

          // If lidar data is present, trigger handler
          if (decoded is Map<String, dynamic> && decoded.containsKey('lidar')) {
            if (onLidarDataReceived != null) {
              onLidarDataReceived!(decoded['lidar']);
            }
          }
        } catch (e) {
          print("Error parsing message: $e");
        }

        // Pass every message to provided callback regardless of content
        onMessage(message);
      },
      onError: (error) {
        print("Error receiving WebSocket message: $error");
      },
      onDone: () {
        print("WebSocket connection closed");
      },
      cancelOnError: true,
    );
  }

  // Gracefully close WebSocket connection and subscription
  void closeConnection() {
    _subscription?.cancel();
    _channel?.sink.close();
    print("WebSocket connection closed");
  }
}
