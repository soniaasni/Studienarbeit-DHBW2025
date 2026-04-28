#!/bin/bash
set -e


# -----------------------------
# Konfiguration
# -----------------------------
ROS_DISTRO="jazzy"
ROS_WS="/root/ros2_ws"
SHARED_ROS2="/root/shared/ros2"
ROS_DOMAIN_ID_FILE="$SHARED_ROS2/ros_domain_id.txt"
MAIN_PYTHON_NODE="$ROS_WS/src/car_system_launch.py"

export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# -----------------------------
# ROS_DOMAIN_ID Handling
# -----------------------------
mkdir -p "$SHARED_ROS2"

if [ ! -f "$ROS_DOMAIN_ID_FILE" ]; then
    echo "0" > "$ROS_DOMAIN_ID_FILE"
fi

export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")

source /opt/ros/$ROS_DISTRO/setup.bash


# -----------------------------
# Build Workspace (nur falls nötig)
# -----------------------------
if [ ! -f "$ROS_WS/install/setup.bash" ]; then
    echo "[ENTRYPOINT] ROS2 Workspace nicht gebaut, starte Build…"
    cd "$ROS_WS"

    rosdep update
    rosdep install \
        --from-path src \
        --ignore-src \
        --rosdistro "$ROS_DISTRO" \
        -y \
        --skip-keys "actionlib catkin message_generation rviz rosparam_shortcuts"

    colcon build \
        --packages-select \
        ros2_bridge \
        car_controller \
        lidar_obstacle_avoidance \
        ultrasonic_sensor \
        custom_msgs

    rm -rf build log
fi

# Strip CRLF from all text/script files in install (handles Windows volume mounts)
find "$ROS_WS/install" -type f \( -name "*.bash" -o -name "*.sh" -o -name "*.py" -o -name "*.dsv" -o -name "*.xml" \) \
    | xargs sed -i 's/\r//' 2>/dev/null || true
# Fix shebanged executables with no extension (e.g. colcon-generated wrapper scripts)
find "$ROS_WS/install" -type f -perm /111 | while read -r f; do
    if head -c 2 "$f" | grep -q $'^#!'; then
        sed -i 's/\r//' "$f"
    fi
done
source "$ROS_WS/install/setup.bash"

echo "[ENTRYPOINT] Starte Car System…"
exec python3 "$MAIN_PYTHON_NODE"

