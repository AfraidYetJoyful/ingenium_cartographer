#!/bin/bash
# Get bag file from args
file=$(realpath "$1")
if [ ! -f "$file" ]; then ### if the file is not found, raise an error and exit the script
  echo "$file is not a file."
  return 1
fi

# Run ros setup
source /opt/ros/noetic/setup.bash
# Run cartographer setup
echo "$HOME"/catkin_ws/install_isolated/setup.bash ### not sure what this line is doing.
source "$HOME"/catkin_ws/install_isolated/setup.bash

roscore & ### execute command roscore in the background and proceed whether or not it has finished running.
sleep 2 ### Pause 2 seconds to allow roscore to launch, and proceed

roslaunch ~/ingenium_cartographer/cartographer_config/display.launch & ### Launch ros with the settings in display.launch (a file composed mainly of links to other config files). Launch it in the background and proceed whether or not it has finished running.
sleep 2 ### Pause to give roslaunch time to start

rosbag play "$file" ### This line displays the .bag file specified in the argument passed to this script when it is run.
sleep 2
