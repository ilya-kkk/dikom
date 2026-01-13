FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
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
