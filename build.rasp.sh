#!/bin/bash
MAKELFAGS="-j1" colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source install/setup.bash
