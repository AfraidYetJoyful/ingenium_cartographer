#!/bin/bash

source /opt/ros/humble/setup.bash

cwd=$(pwd) #AB save the current working directory
file=$(realpath "$1")

ros2 launch lio_sam run.launch.py &
sleep 5

cd /home/Documents/Data

ros2 bag play $file

# Return to original directory
cd "$cwd"

echo "Bag fully processed, press enter to exit"
read -r ### Pause until reading any character. Once a character is read, progress to the next line (that is, terminate the program).
