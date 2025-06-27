#!/bin/bash

#AB Bash script to reinstall all the scripts and apps necessary for the project. Run on a fresh Ubuntu install.



#---------------------------------------------INSTALL BASIC PACKAGES---------------------------------------------
echo -ne "Installing base packages...\n"
sleep 1

sudo apt update
sudo apt upgrade
sudo apt install htop #AB Disk space monitor
sudo apt install openssh-server #AB SSH client
sudo apt install blender #AB Install blender (a 3D modeling software)
sudo apt install snapd #AB A package manager
sudo apt install gnome-tweaks #AB An OS customization tool
sudo snap install --classic code #AB Visual Studio Code, a git-integrated IDE for basically all computer languages
sudo snap install cloudcompare #AB Install CloudCompare (a point-cloud processing software)
sudo apt-get install gnome-keyring #AB Install a secure cryptographic library needed by VS Code
sudo apt install rpi-imager #AB a software for burning OSes onto SD cards for use in a Raspberry Pi
sudo snap refresh firefox #AB Update the default-installed firefox to the latest version

sudo apt-get install git #AB Install and then configure git (a source control software for coders)
git config --global user.email "ingenium.lidar@outlook.com"
git config --global user.name "Ingenium-LiDAR"

sudo apt-get install gnome-keyring #AB Install a secure cryptographic library needed by VS Code



#---------------------------------------------INSTALL ROS2 Jazzy---------------------------------------------


if [ -f "./Install_Jazzy.sh" ]; #AB If the ROS2 Jazzy installer script file exists...
then 
  ./Install_Jazzy.sh #AB ...execute it, to install ROS Jazzy
else #AB otherwise skip it.
    echo "./Install_Jazzy.sh was not found. Skipping ROS Jazzy install..."
fi



#---------------------------------------------INSTALL HARDWARE DRIVERS---------------------------------------------


sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-jazzy-velodyne #AB Install the IMU driver. It's in a stack hosted (I believe) on the ROS website.
sudo apt-get install ros-jazzy-microstrain-inertial-driver #AB Install the IMU driver. Turns out that the these drivers are now maintained as part of a built-in ROS package manager! This should make things easier for future updates.



#---------------------------------------------CLEANUP---------------------------------------------


echo -ne "Running sudo apt autoremove:\n"
sudo apt autoremove #AB Remove all files not needed in the system. Frees up a variable amount of space (on the Jun 24, 2025 reinstall, I had superfluous firmware. You never know...)
