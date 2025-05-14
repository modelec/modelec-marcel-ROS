#!/bin/bash
sleep 5

source /opt/ros/jazzy/setup.bash
source /home/acki/ros2_ws/install/setup.bash

export RCL_LOG_LEVEL=info
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/acki/ros2_ws/fastdds_setup.xml
export ROS_DOMAIN_ID=128

# Lance ton launch file
exec ros2 launch modelec_core test.modelec.launch.py