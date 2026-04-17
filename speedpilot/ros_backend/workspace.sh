#!/bin/bash
set -e

ROS_DISTRO="jazzy"
source /opt/ros/$ROS_DISTRO/setup.bash

# Minimale Abhängigkeiten installieren
apt-get update && apt-get install -y --no-install-recommends \
    gnupg curl libpcap-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

cd /root/ros2_ws

# ROS-Deps installieren
rosdep update || echo "Warning: Failed to update rosdep"
rosdep install --from-path src --rosdistro $ROS_DISTRO -y \
    --skip-keys "actionlib catkin message_generation rviz rosparam_shortcuts" \
    || echo "Warning: Some dependencies could not be resolved"

# Pakete bauen
colcon build --packages-select ros2_bridge car_controller lidar_obstacle_avoidance ultrasonic_sensor custom_msgs || echo "Warning: Build failed"
source install/setup.bash

echo "Workspace setup completed!"