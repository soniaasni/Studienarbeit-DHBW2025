#!/bin/bash
set -e

ROS_DISTRO="jazzy"

source /opt/ros/$ROS_DISTRO/setup.bash

cd /root/ros2_ws

echo "Installing ROS 2 dependencies..."
rosdep update || echo "Warning: rosdep update failed"
rosdep install --from-path src --rosdistro $ROS_DISTRO -y \
    --skip-keys "actionlib catkin message_generation rviz rosparam_shortcuts" \
    || echo "Warning: Some dependencies could not be resolved"

echo "Building workspace..."
colcon build --packages-select ros2_bridge car_controller lidar_obstacle_avoidance ultrasonic_sensor custom_msgs
source install/setup.bash

echo "Workspace setup complete."
