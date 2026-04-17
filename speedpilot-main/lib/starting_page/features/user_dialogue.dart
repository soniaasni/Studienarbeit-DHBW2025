import 'package:flutter/material.dart';
import "package:speedpilot/models/card_list.dart";

// Dialog widget to add a new device (vehicle) entry
class AddUserDialogue extends StatefulWidget {
  final Function(CardList) addDeviceCard; // Callback to pass new device to parent

  const AddUserDialogue(this.addDeviceCard, {super.key});

  @override
  _AddUserDialogueState createState() => _AddUserDialogueState();
}

class _AddUserDialogueState extends State<AddUserDialogue> {
  // Controllers for name and IP address input fields
  final TextEditingController nameController = TextEditingController();
  final TextEditingController ipaddressController = TextEditingController();

  @override
  Widget build(BuildContext context) {
    // Helper function to build styled text fields
    Widget buildTextField(String hint, TextEditingController controller) {
      return Container(
        margin: const EdgeInsets.symmetric(vertical: 8),
        child: TextField(
          style: const TextStyle(color: Colors.white),
          decoration: InputDecoration(
            labelText: hint,
            labelStyle: const TextStyle(color: Colors.white70),
            enabledBorder: const OutlineInputBorder(
              borderSide: BorderSide(color: Colors.white70),
            ),
            focusedBorder: const OutlineInputBorder(
              borderSide: BorderSide(color: Colors.redAccent),
            ),
          ),
          controller: controller,
        ),
      );
    }

    return Column(
      mainAxisSize: MainAxisSize.min, // Wrap content vertically
      children: [
        // Title
        const Text(
          "Add Vehicle",
          style: TextStyle(
            fontWeight: FontWeight.bold,
            fontSize: 22,
            color: Colors.white,
          ),
        ),
        // Input fields
        buildTextField('Name', nameController),
        buildTextField('IP Address', ipaddressController),
        const SizedBox(height: 16),

        // Add button
        ElevatedButton(
          style: ElevatedButton.styleFrom(
            backgroundColor: Colors.red,
            foregroundColor: Colors.white,
          ),
          onPressed: () {
            // Create new CardList object with prefixed ws:// IP address
            final device = CardList(
              nameController.text,
              "ws://${ipaddressController.text}",
            );
            widget.addDeviceCard(device); // Pass to parent
            Navigator.pop(context); // Close the dialog
          },
          child: const Text('Add'),
        ),
      ],
    );
  }
}
