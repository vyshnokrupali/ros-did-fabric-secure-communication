FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt update && apt install -y \
    python3 \
    python3-pip \
    curl \
    gnupg2 \
    lsb-release \
    nano

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt update && apt install -y ros-humble-ros-base

# Python deps
RUN pip3 install requests

WORKDIR /app

COPY . /app

RUN chmod +x entrypoint.sh

ENTRYPOINT ["./entrypoint.sh"]
