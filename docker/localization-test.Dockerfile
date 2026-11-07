FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions \
    ros-humble-robot-localization \
    ros-humble-launch-testing ros-humble-launch-testing-ament-cmake \
    ros-humble-ament-lint-auto ros-humble-ament-lint-common \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --uid 10001 rover
COPY run-localization-test.sh /usr/local/bin/run-localization-test
RUN chmod 755 /usr/local/bin/run-localization-test
ENV PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=197 ROS_LOG_DIR=/tmp/ros-logs
USER rover
WORKDIR /tmp
CMD ["/usr/local/bin/run-localization-test"]
