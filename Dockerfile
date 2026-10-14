FROM ros:humble

ARG ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        curl \
        git \
        i2c-tools \
        libcurl4-openssl-dev \
        libusb-1.0-0-dev \
        nlohmann-json3-dev \
        pkg-config \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        ros-${ROS_DISTRO}-backward-ros \
        ros-${ROS_DISTRO}-depthimage-to-laserscan \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-geographic-msgs \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-nmea-msgs \
        ros-${ROS_DISTRO}-pointcloud-to-laserscan \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-rplidar-ros \
        ros-${ROS_DISTRO}-rtcm-msgs \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-zed-msgs \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

RUN python3 -m pip install --no-cache-dir sparkfun-qwiic-icm20948

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /root/.bashrc

WORKDIR /ros2_ws
