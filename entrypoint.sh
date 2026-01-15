#!/bin/bash

source /opt/ros/humble/setup.bash

if [ "$ROLE" = "publisher" ]; then
    python3 ros_publisher.py
else
    python3 ros_subscriber.py
fi
