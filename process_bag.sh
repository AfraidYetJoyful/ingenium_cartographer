#!/bin/bash

source /opt/ros/humble/setup.bash

BOLD_CYAN='\e[1;36m'
NC='\033[0m'

cwd=$(pwd) #AB save the current working directory
file=$(realpath "$1")

ros2 launch lio_sam run.launch.py &
sleep 5

cd /home/Documents/Data

ros2 bag play $file --log-level debug

# Return to original directory
cd "$cwd"

echo "process_bag.sh has finished running. Press enter to exit."
read -r ### Pause until reading any character. Once a character is read, progress to the next line (that is, terminate the program).
