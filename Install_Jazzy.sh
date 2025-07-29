#!/bin/bash

#AB ROS Jazzy Installation Script, copied from https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
#AB to use ROS in a given terminal session, run source /opt/ros/jazzy/setup.bash
cwd=$(pwd)

echo "Updating apt..."
sleep 1
sudo apt update
sudo apt upgrade
sudo apt autoremove

echo "Installing universe repository..."
sleep 1
sudo apt install software-properties-common
sudo add-apt-repository universe

echo "Configuring system..."
sleep 1
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "Updating apt a second time..."
sleep 1
sudo apt update
sudo apt upgrade

echo "Installing ros-jazzy-desktop..."
sleep 1
sudo apt install ros-jazzy-desktop

echo "Installing rosbag2..."
sleep 1
sudo apt-get install ros-jazzy-rosbag2

echo 'alias run_jazzy="source /opt/ros/jazzy/setup.bash"' >> ~/.bashrc #AB add the alias run_jazzy to the system ~/.bashrc file.

echo "ROS2 Jazzy installation complete."
sleep 1

cd $cwd