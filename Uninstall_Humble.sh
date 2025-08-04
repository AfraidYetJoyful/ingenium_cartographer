#!/bin/bash

#FK meant for Ubuntu 22.04
#FK from this website: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
#FK has never been tested

sudo apt remove ~nros-humble-* && sudo apt autoremove
sudo apt remove ros2-apt-source
sudo apt update
sudo apt autoremove
sudo apt upgrade # Consider upgrading for packages previously shadowed.

echo “To complete this process, please remove the following lines from your .bashrc file:”
echo "# source ros2 bash file"
echo “source /opt/ros/humble/setup.bash”
