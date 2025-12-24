#!/bin/bash
rm -rf build log install
colcon build --packages-select turtle_square || { echo "Build failed!"; exit 1; }

source install/setup.bash

ros2 run turtle_square turtle_square_node 

wait
