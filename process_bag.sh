#!/bin/bash

source /opt/ros/humble/setup.bash

file=$(realpath "$1")

ros2 launch lio_sam run.launch.py &
sleep 5

cd /home/Documents/Data

ros2 bag play $file