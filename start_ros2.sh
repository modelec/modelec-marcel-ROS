#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /home/modelec/modelec-marcel-ROS/install/setup.bash

export RCL_LOG_LEVEL=info
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/modelec/modelec-marcel-ROS/fastdds_setup.xml
export ROS_DOMAIN_ID=128

exec ros2 launch modelec_core modelec.launch.py
