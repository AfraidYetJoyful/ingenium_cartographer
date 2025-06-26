#!/bin/bash

#AB Bash script to reinstall all the scripts and apps necessary for the project. Run on a fresh Ubuntu install.



#---------------------------------------------INSTALL BASIC PACKAGES---------------------------------------------


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



#---------------------------------------------INSTALL ROS2 Jazzy---------------------------------------------


if [ -f "./Install_Jazzy.sh" ]; #AB If the ROS2 Jazzy installer script file exists...
then 
  ./Install_Jazzy.sh #AB ...execute it, to install ROS Jazzy
else #AB otherwise skip it.
    echo "./Install_Jazzy.sh was not found. Skipping ROS Jazzy install..."
fi



#---------------------------------------------INSTALL HARDWARE DRIVERS---------------------------------------------


#AB Make and go to a new directory called ingenium_cartographer_scripts in the ~ directory.
cd ~
mkdir ingenium_cartographer_scripts
cd ingenium_cartographer_scripts
mkdir drivers #AB make and navigate to a new directory called "drivers" in ingenium_cartographer_scripts
cd drivers


function clone_at_commit() { #AB Copied straight from install.sh
  url=$1
  commit=$2
  git clone "$url"
  cd "$(basename "$url")" || exit
  git reset --hard "$commit"
  git pull
  cd .. || exit
}


clone_at_commit https://github.com/ros-drivers/velodyne/ 0f2a3bb1dde4fa91cbafad8a3f9f89b66c2a1350 #AB Clone the velodyne driver. This was the most recent commit on June 26, 2025, when I wrote this comment. No need that this should necessarily stay the default, but if you want to upgrade check for compatibility first and save this commit ID so you can revert if you need to.
#AB Note that Johannes was unsatisfied with the ROS1 version of this ^ and wrote his own version, which is on his GitHub. If things start breaking I will talk to him about what he did and why.


sudo apt-get update && sudo apt-get install ros-jazzy-microstrain-inertial-driver #AB Install the IMU driver. Turns out that the these drivers are now maintained as part of a built-in ROS package manager! This should make things easier for future updates.


echo "Running sudo apt autoremove:"
sudo apt autoremove #AB Remove all files not needed in the system. Frees up a variable amount of space (on the Jun 24, 2025 reinstall, I had superfluous firmware. You never know...)
