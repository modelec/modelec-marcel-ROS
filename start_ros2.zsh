#!/bin/bash
set -e

source /opt/ros/jazzy/setup.zsh
source ~/Modelec-ROS2/install/setup.zsh

export RCL_LOG_LEVEL=info

exec ros2 launch modelec_core modelec.launch.py "$@"
