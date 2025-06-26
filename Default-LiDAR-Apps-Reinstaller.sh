#!/bin/bash

#AB Bash script to reinstall all the scripts and apps necessary for the project. Run on a fresh Ubuntu install.

sudo apt update
sudo apt upgrade
sudo apt install htop #AB Disk space monitor
sudo apt install openssh-server #AB SSH client
sudo apt install blender #AB Install blender (a 3D modeling software)
sudo apt install snapd #AB A package manager
sudo snap install --classic code #AB Visual Studio Code, a git-integrated IDE for basically all computer languages
sudo snap install cloudcompare #AB Install CloudCompare (a point-cloud processing software)
sudo apt-get install gnome-keyring #AB Install a secure cryptographic library needed by VS Code
sudo apt install rpi-imager #AB a software for burning OSes onto SD cards for use in a Raspberry Pi
sudo snap refresh firefox #AB Update the default-installed firefox to the latest version

sudo apt-get install git #AB Install and then configure git (a source control software for coders)
git config --global user.email "ingenium.lidar@outlook.com"
git config --global user.name "Ingenium-LiDAR"

sudo apt-get install gnome-keyring #AB Install a secure cryptographic library needed by VS Code

sudo apt autoremove #AB Remove all files not needed in the system. Frees up a variable amount of space (on the Jun 24, 2025 reinstall, I had superfluous firmware. You never know...)

if [ -f "./Install_Jazzy.sh" ]; #AB If file exists...
then 
  ./Install_Jazzy.sh #AB ...execute it
else #AB otherwise skip it.
    echo "./Install_Jazzy.sh was not found. Skipping ROS Jazzy install..."
fi

