#!/bin/bash
MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential --cmake-args -DBUILD_FOR_RPI=ON

source install/setup.bash
