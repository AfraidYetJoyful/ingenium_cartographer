#!/bin/bash

#FK from this documentation: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
#FK meant to be used on Ubuntu 22.04

#FK has worked on Finn's computer, 8/1/2025, windows subsystem for linux with Ubuntu 22.04.5 LTS

sudo apt update
sudo apt upgrade
sudo apt autoremove


echo “checking for UTF-8...“
sleep 2
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
echo "done checking for UTF-8. If not UTF-8, kill the process within 5 seconds."
sleep 5

#then run the following to set things up

echo "Beginning setup..."
sleep 2

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade

echo "Installing ros humble desktop..."
sleep 2
sudo apt install ros-humble-desktop
echo "Installing ros humble dev tools..."
sleep 2
sudo apt install ros-dev-tools

echo "Setting up the environment by sourcing the .bash file"
echo "Assuming bash shell. Else, please kill this process within 5 seconds and edit the currently running script."
sleep 5
source /opt/ros/humble/setup.bash

cd ~
echo "Adding alias run_humble to system ~/.bashrc file"
echo 'alias run_humble="source /opt/ros/humble/setup.bash"' >> ~/.bashrc

echo "All done!"

