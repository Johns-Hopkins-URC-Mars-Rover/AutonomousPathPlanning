FROM ros:humble

ARG ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1

# Build tools, hardware utilities, ROS runtime dependencies, and test tools for
# the physical-rover stack. Gazebo Classic is intentionally omitted because its
# Humble packages are not published for this ARM64 base image.
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
        python3-numpy \
        python3-pip \
        python3-pytest \
        python3-rosdep \
        python3-serial \
        python3-vcstool \
        python3-yaml \
        ros-${ROS_DISTRO}-ament-cmake-auto \
        ros-${ROS_DISTRO}-ament-lint-common \
        ros-${ROS_DISTRO}-backward-ros \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-geographic-msgs \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-imu-filter-madgwick \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-launch-testing \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nmea-msgs \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-zed-msgs \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

# Hardware Python libraries imported by the active L76X and ICM-20948 drivers.
RUN python3 -m pip install \
        Jetson.GPIO \
        smbus2 \
        sparkfun-qwiic-icm20948

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /root/.bashrc

WORKDIR /ros2_ws

CMD ["bash"]
