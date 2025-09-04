#!/bin/bash
set -e

# Check if ROS_DOMAIN_ID is set and not empty. If so, apply it.
if [ -n "${ROS_DOMAIN_ID}" ]; then
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} found. Adding to .bashrc and sourcing."
    echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> ~/.bashrc
    source ~/.bashrc
else
    echo "ROS_DOMAIN_ID not set. Using default."
fi

# Source ROS 2 and workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"