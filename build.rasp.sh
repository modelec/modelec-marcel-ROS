#!/bin/bash
MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug

source install/setup.bash
