#!/bin/zsh
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug

source "install/setup.zsh"
