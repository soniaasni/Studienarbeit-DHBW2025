# Multi-Stage Build für kleineren Container

# Stage 1: Builder
FROM ros:jazzy AS builder

LABEL maintainer="MaxDomitrovic<max@domitrovic.com>"

ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Minimale Updates und Installationen für Build
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
    wget curl git build-essential \
    python3-argcomplete python3-colcon-common-extensions python3-networkx python3-pip python3-rosdep python3-vcstool \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py \
    ros-jazzy-rviz2 ros-jazzy-rqt-reconfigure ros-jazzy-rosparam-shortcuts && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS Setup
RUN rosdep update && \
    echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Kopiere und baue Workspace
COPY ros2_ws/src/ /root/ros2_ws/src/
WORKDIR /root/ros2_ws
RUN rosdep install --from-path src --ignore-src --rosdistro jazzy -y --skip-keys "actionlib catkin message_generation rviz rosparam_shortcuts" && \
    colcon build --packages-select ros2_bridge car_controller lidar_obstacle_avoidance ultrasonic_sensor custom_msgs && \
    rm -rf build log

# Stage 2: Runtime (minimal)
FROM ros:jazzy

LABEL maintainer="MaxDomitrovic<max@domitrovic.com>"

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Minimale Installationen für Runtime
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
    libserial-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Kopiere gebaute Artefakte
COPY --from=builder /root/ros2_ws/install /root/ros2_ws/install
COPY --from=builder /root/.bashrc /root/.bashrc

# Setup-Skripte
COPY docker/entrypoint.sh /root/
COPY docker/bash_aliases.txt /root/.bashrc_aliases
RUN chmod +x /root/entrypoint.sh && \
    cat /root/.bashrc_aliases >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Verzeichnisse
RUN mkdir -p /etc/udev/rules.d /root/ros2_ws/src

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["/bin/bash", "-c", "tail -f /dev/null"]
