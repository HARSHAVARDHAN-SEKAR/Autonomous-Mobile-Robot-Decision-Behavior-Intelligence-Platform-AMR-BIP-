# Autonomous Mobile Robot Decision & Behavior Intelligence Platform - reproducible dev/sim image
# ROS 2 Humble + Gazebo Classic 11 on Ubuntu 22.04
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# --- system + ROS dependencies (pinned distro: humble) ----------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # build
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    build-essential \
    # simulation
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    # visualization + manual driving
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    # runtime python deps
    python3-numpy \
    python3-yaml \
    # tools
    tmux nano less \
    && rm -rf /var/lib/apt/lists/*

# --- workspace ---------------------------------------------------------------
WORKDIR /ws
# source code is bind-mounted at runtime (see docker-compose.yml / RUN_GUIDE),
# so edits on the host are instantly visible inside the container.

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# make every interactive shell ROS-aware
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
