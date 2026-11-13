# Gazebo Classic 11 / Humble compatibility environment. On Apple Silicon build
# with --platform linux/amd64; the repository's plugins target Classic.
FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential python3-colcon-common-extensions python3-pytest \
    python3-pytest-cov python3-numpy python3-yaml \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-ros2-controllers ros-humble-robot-state-publisher \
    ros-humble-robot-localization ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-launch-testing ros-humble-launch-testing-ament-cmake \
    ros-humble-ament-lint-auto ros-humble-ament-lint-common \
    xvfb xauth mesa-utils libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --uid 10001 rover
ENV LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 ROS_LOCALHOST_ONLY=1
USER rover
WORKDIR /tmp/rover_ws
CMD ["bash"]
