#!/bin/bash
MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential # --cmake-args -DCMAKE_BUILD_TYPE=Debug

source install/setup.bash
