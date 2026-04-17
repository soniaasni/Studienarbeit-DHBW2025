import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

// SettingsList widget with toggle switches (currently only Gyroscope is functional)
class SettingsList extends StatefulWidget {
  final Function(bool) onSwitchChanged;

  const SettingsList({Key? key, required this.onSwitchChanged}) : super(key: key);

  @override
  _SettingsList createState() => _SettingsList();
}

class _SettingsList extends State<SettingsList> {
  bool _enable = false; // Stores current gyro switch state

  @override
  void initState() {
    super.initState();
    _loadSwitchValue(); // Load stored value when widget initializes
  }

  // Load gyro setting from SharedPreferences
  Future<void> _loadSwitchValue() async {
    final prefs = await SharedPreferences.getInstance();
    final storedValue = prefs.getBool('gyro') ?? false;
    setState(() {
      _enable = storedValue;
    });
  }

  // Save new gyro setting to SharedPreferences
  Future<void> _saveSwitchValue(bool value) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setBool('gyro', value);
  }

  // List of settings (only the first item is active)
  final List<CardOption> settings = [
    CardOption(
      title: "Gyroscope",
      color: const Color.fromARGB(250, 35, 35, 35),
      fontColor: Colors.white,
    ),
    // The remaining items are placeholders
    CardOption(
      title: "Steering",
      color: const Color.fromARGB(250, 35, 35, 35),
      fontColor: Colors.white,
    ),
    CardOption(
      title: "Steering",
      color: const Color.fromARGB(250, 35, 35, 35),
      fontColor: Colors.white,
    ),
    CardOption(
      title: "Steering",
      color: const Color.fromARGB(250, 35, 35, 35),
      fontColor: Colors.white,
    ),
    CardOption(
      title: "Steering",
      color: const Color.fromARGB(250, 35, 35, 35),
      fontColor: Colors.white,
    ),
    CardOption(
      title: "Steering",
      color: const Color.fromARGB(250, 35, 35, 35),
      fontColor: Colors.white,
    ),
    CardOption(
      title: "Steering",
      color: const Color.fromARGB(250, 35, 35, 35),
      fontColor: Colors.white,
    ),
  ];

  @override
  Widget build(BuildContext context) {
    return ListView.builder(
      itemCount: settings.length, // Total number of setting cards
      itemBuilder: (context, index) {
        final setting = settings[index];

        return Container(
          margin: EdgeInsets.only(
            top: MediaQuery.of(context).size.height * 0.1,
            left: MediaQuery.of(context).size.height * 0.2,
            right: MediaQuery.of(context).size.height * 0.2,
          ),
          height: MediaQuery.of(context).size.height * 0.15,
          child: Card(
            color: setting.color,
            margin: const EdgeInsets.symmetric(vertical: 0, horizontal: 0),
            child: Padding(
              padding: const EdgeInsets.all(15),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  // Show title only for Gyroscope and first Steering card
                  if (index == 0 || index == 1)
                    Text(
                      setting.title,
                      style: TextStyle(
                        color: setting.fontColor,
                        fontSize: 18,
                        fontWeight: FontWeight.bold,
                      ),
                    ),

                  // Show CustomSwitch only for Gyroscope
                  if (index == 0)
                    CustomSwitch(
                      value: _enable,
                      onChanged: (bool val) {
                        setState(() {
                          _enable = val;
                        });
                        _saveSwitchValue(val); // Save to preferences
                        widget.onSwitchChanged(val); // Notify parent
                      },
                    ),
                ],
              ),
            ),
          ),
        );
      },
    );
  }
}

// Class to represent settings card data
class CardOption {
  final String title;
  final Color color;
  final Color fontColor;

  CardOption({
    required this.title,
    required this.color,
    required this.fontColor,
  });
}

// Custom switch widget with animation
class CustomSwitch extends StatefulWidget {
  final bool value;
  final ValueChanged<bool> onChanged;

  const CustomSwitch({Key? key, required this.value, required this.onChanged}) : super(key: key);

  @override
  _CustomSwitchState createState() => _CustomSwitchState();
}

class _CustomSwitchState extends State<CustomSwitch> with SingleTickerProviderStateMixin {
  late AnimationController _animationController;
  late Animation<Alignment> _circleAnimation;

  @override
  void initState() {
    super.initState();

    // Set up animation controller
    _animationController = AnimationController(
      duration: Duration(milliseconds: 60),
      vsync: this,
    );

    // Define animation direction based on current value
    _circleAnimation = AlignmentTween(
      begin: widget.value ? Alignment.centerRight : Alignment.centerLeft,
      end: widget.value ? Alignment.centerLeft : Alignment.centerRight,
    ).animate(CurvedAnimation(
      parent: _animationController,
      curve: Curves.linear,
    ));

    // Set initial position of the switch
    _animationController.value = widget.value ? 1.0 : 0.0;
  }

  @override
  void didUpdateWidget(CustomSwitch oldWidget) {
    super.didUpdateWidget(oldWidget);

    // Rebuild animation if value changes
    if (widget.value != oldWidget.value) {
      _circleAnimation = AlignmentTween(
        begin: widget.value ? Alignment.centerLeft : Alignment.centerRight,
        end: widget.value ? Alignment.centerRight : Alignment.centerLeft,
      ).animate(CurvedAnimation(
        parent: _animationController,
        curve: Curves.linear,
      ));

      // Play animation in the correct direction
      widget.value ? _animationController.forward() : _animationController.reverse();
    }
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _animationController,
      builder: (context, child) {
        return GestureDetector(
          onTap: () {
            final newValue = !widget.value;
            widget.onChanged(newValue); // Toggle the switch
          },
          child: Container(
            width: 45.0,
            height: 28.0,
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(24.0),
              color: widget.value ? Colors.green : Colors.grey,
            ),
            child: Padding(
              padding: const EdgeInsets.all(2.0),
              child: Align(
                alignment: widget.value ? Alignment.centerRight : Alignment.centerLeft,
                child: Container(
                  width: 20.0,
                  height: 20.0,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    color: Colors.white,
                  ),
                ),
              ),
            ),
          ),
        );
      },
    );
  }
}
