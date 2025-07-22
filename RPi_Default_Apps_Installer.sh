#!/bin/bash

#AB Run on a clean Ubuntu Server 24.04.2 LTS system

cwd = $(pwd)

#FK updates and upgrades
sudo apt update
sudo apt upgrade
sudo apt update --fix-missing
sudo apt upgrade --fix-missing
sudo apt-get update
sudo apt-get upgrade
sudo apt-get update --fix-missing
sudo apt-get upgrade --fix-missing

sudo apt install network-manager #AB add utility for managing networks
sudo apt install net-tools #AB add another utility for managing networks
sudo apt-get install git #AB install git, just in case it is not already installed

mkdir -p ~/Documents/GitHub #Create the GitHub directory in the ~/Documents directory. If ~/Documents does not exist, the -p flag creates it also.
cd ~/Documents/GitHub

#AB Clone the ingenium_cartographer repository if it does not already exist
if ! [ -d "ingenium_cartographer" ]; then
    git clone https://github.com/JohannesByle/ingenium_cartographer.git
fi

cd ingenium_cartographer
git switch jazzy


#AB Install ROS Jazzy
chmod +x Install_Jazzy.sh #AB make Install_Jazzy.sh executable and run it
./Install_Jazzy.sh 




sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-jazzy-velodyne #AB Install the IMU driver. It's in a stack hosted (I believe) on the ROS website.
sudo apt-get install ros-jazzy-microstrain-inertial-driver #AB Install the IMU driver. Turns out that the these drivers are now maintained as part of a built-in ROS package manager! This should make things easier for future updates.



cd $cwd #AB return to the original directory
echo "RPi_Default_Apps_Installer.sh has finished running now."