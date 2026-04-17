# Multi-Stage Build für kleineren Container

# Stage 1: Builder
FROM ros:jazzy AS builder

LABEL maintainer="MaxDomitrovic<max@domitrovic.com>"

ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Minimale Updates und Installationen für Build
RUN apt-get update -q && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    wget curl git build-essential python3-pip python3-rosdep \
    python3-argcomplete python3-colcon-common-extensions python3-networkx python3-vcstool && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS Setup
RUN rosdep update && \
    echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Kopiere Workspace Source
COPY ros2_ws/src/ /root/ros2_ws/src/

# Stage 2: Runtime (minimal)
FROM ros:jazzy

LABEL maintainer="MaxDomitrovic<max@domitrovic.com>"

ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Minimale Installationen für Runtime
RUN apt-get update -q && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    wget curl git python3-pip python3-rosdep \
    python3-argcomplete python3-colcon-common-extensions python3-networkx python3-vcstool \
    libserial-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Kopiere Source vom Builder
COPY --from=builder /root/ros2_ws/src /root/ros2_ws/src

# Setup-Skripte
COPY docker/bash_aliases.txt /root/.bashrc_aliases

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    cat /root/.bashrc_aliases >> /root/.bashrc

# Verzeichnisse
RUN mkdir -p /etc/udev/rules.d /root/ros2_ws/build /root/ros2_ws/install /root/ros2_ws/log

# Kopiere entrypoint script
COPY docker/entrypoint.sh /root/entrypoint.sh
RUN dos2unix /root/entrypoint.sh 2>/dev/null || sed -i 's/\r$//' /root/entrypoint.sh && chmod +x /root/entrypoint.sh

WORKDIR /root/ros2_ws

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["/bin/bash", "-c", "tail -f /dev/null"]
