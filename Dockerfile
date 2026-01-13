FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-nav2-msgs \
    ros-humble-nav2-util \
    ros-humble-nav2-controller \
    ros-humble-nav2-core \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-planner \
    ros-humble-nav2-smoother \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-velocity-smoother \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-dwb-core \
    ros-humble-nav2-navfn-planner \
    ros-humble-slam-toolbox \
  && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

WORKDIR /workspace
RUN mkdir -p /workspace/src

COPY . /workspace/src/dikom

RUN source /opt/ros/humble/setup.bash \
  && colcon build --symlink-install --merge-install

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
