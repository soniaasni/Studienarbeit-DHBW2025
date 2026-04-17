#!/bin/bash
set -e

ROS_DISTRO="jazzy"

export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

ROS_WS="/root/ros2_ws"
SHARED_ROS2="/root/shared/ros2"

ROS_DOMAIN_ID_FILE="$SHARED_ROS2/ros_domain_id.txt"

if [ ! -f "$ROS_DOMAIN_ID_FILE" ]; then
    mkdir -p "$SHARED_ROS2"
    echo "0" > "$ROS_DOMAIN_ID_FILE"
fi

export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")

if ! grep -q "export ROS_DOMAIN_ID" /root/.bashrc; then
  echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> /root/.bashrc
fi

source /opt/ros/$ROS_DISTRO/setup.bash

if [ ! -d "$ROS_WS/install" ]; then
    echo "Building ROS2 workspace..."
    cd $ROS_WS
    rosdep update
    rosdep install --from-path src --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys "actionlib catkin message_generation rviz rosparam_shortcuts"
    colcon build --packages-select ros2_bridge car_controller lidar_obstacle_avoidance ultrasonic_sensor custom_msgs
    rm -rf build log
fi

if [ -f "$ROS_WS/install/setup.bash" ]; then
    source $ROS_WS/install/setup.bash
fi

source /root/.bashrc

cd $ROS_WS

exec "$@"
