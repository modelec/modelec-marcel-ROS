MAKEFLAGS="-j2" colcon build --packages-skip modelec_com --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash