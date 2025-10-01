import 'package:flutter/material.dart';
import 'package:speedpilot/driving_page/gyroscope_page.dart';
import 'package:speedpilot/driving_page/joystick_page.dart';
import 'package:shared_preferences/shared_preferences.dart';

// Stateful widget for a custom image carousel with clickable pages
class CustomCarousel extends StatefulWidget {
  @override
  _CustomCarouselState createState() => _CustomCarouselState();
}

class _CustomCarouselState extends State<CustomCarousel> {
  // PageController with reduced viewport to show part of neighboring cards
  final PageController _pageController = PageController(viewportFraction: 0.7);
  int _currentPage = 0;

  // List of image paths and captions used in the carousel
  final List<Map<String, String>> images = [
    {'image': 'assets/images/nomap.png', 'caption': 'Without Map'},
    {'image': 'assets/images/lidar.jpg', 'caption': 'H222'},
    {'image': 'assets/images/PfuschMobil.png', 'caption': 'Bild 3'},
  ];

  @override
  void initState() {
    super.initState();

    // Listen to page scroll and update _currentPage for indicator dot highlighting
    _pageController.addListener(() {
      int nextPage = _pageController.page!.round();
      if (_currentPage != nextPage) {
        setState(() {
          _currentPage = nextPage;
        });
      }
    });
  }

  // Optional method to set 'gyro' preference to false
  Future<void> _setGyroToFalse() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setBool('gyro', false);
  }

  @override
  void dispose() {
    // Dispose controller when widget is removed from widget tree
    _pageController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        SizedBox(height: 40),
        // Carousel content
        Expanded(
          child: PageView.builder(
            controller: _pageController,
            itemCount: images.length,
            itemBuilder: (context, index) {
              return GestureDetector(
                onTap: () {
                  // Navigate based on which card is tapped
                  if (index != 0) {
                    Navigator.push(
                      context,
                      MaterialPageRoute(
                        builder: (context) => GyroscopePage(
                          imagePath: images[index]['image']!,
                        ),
                      ),
                    );
                  }
                  if (index == 0) {
                    Navigator.push(
                      context,
                      MaterialPageRoute(
                        builder: (context) => JoystickPage(),
                      ),
                    );
                  }
                },
                child: buildImage(index),
              );
            },
          ),
        ),
        SizedBox(height: 10),
        // Navigation dots at the bottom
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: List.generate(images.length, (index) {
            return buildDot(index);
          }),
        ),
      ],
    );
  }

  // Build each carousel item (image with caption)
  Widget buildImage(int index) {
    return AnimatedContainer(
      duration: Duration(milliseconds: 300),
      margin: EdgeInsets.symmetric(
        vertical: MediaQuery.of(context).size.height * 0.22,
        horizontal: 10,
      ),
      child: Column(
        children: [
          Card(
            elevation: 8.0,
            shadowColor: Colors.black.withOpacity(0.5),
            color: const Color.fromARGB(200, 35, 35, 35),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(15.0),
            ),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(15.0),
              child: SizedBox(
                height: MediaQuery.of(context).size.height * 0.3,
                width: MediaQuery.of(context).size.width * 0.6,
                child: Image.asset(
                  images[index]['image']!,
                  fit: BoxFit.cover,
                ),
              ),
            ),
          ),
          SizedBox(height: 10),
          Text(
            images[index]['caption']!,
            style: TextStyle(
              color: Colors.white,
              fontSize: 20,
              fontWeight: FontWeight.bold,
            ),
          ),
        ],
      ),
    );
  }

  // Build the dot indicators at the bottom of the carousel
  Widget buildDot(int index) {
    return GestureDetector(
      onTap: () {
        // Animate to selected page when dot is tapped
        _pageController.animateToPage(
          index,
          duration: Duration(milliseconds: 300),
          curve: Curves.easeInOut,
        );
      },
      child: Container(
        width: 12,
        height: 12,
        margin: EdgeInsets.symmetric(horizontal: 8, vertical: 50),
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          color: _currentPage == index ? Colors.white : Colors.grey,
        ),
      ),
    );
  }
}
