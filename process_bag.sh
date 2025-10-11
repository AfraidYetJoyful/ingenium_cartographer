#!/bin/bash

source /opt/ros/humble/setup.bash



cwd=$(pwd) #AB save the current working directory
file=$(realpath "$1")

ros2 launch lio_sam run.launch.py &
sleep 5

cd ~/Documents/Data

ros2 bag play $file --log-level debug

# Return to original directory
cd "$cwd"

echo -e '\e[1;36mprocess_bag.sh has finished running. Press enter to exit.\033[0m'
read -r ### Pause until reading any character. Once a character is read, progress to the next line (that is, terminate the program).
