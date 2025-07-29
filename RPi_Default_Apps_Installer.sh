#!/bin/bash

#AB Run on a clean Ubuntu Server 24.04.2 LTS system

cwd=$(pwd)


#---------------------------------------------UPDATE THE SYSTEM AND INSTALL PACKAGES---------------------------------------------


#FK updates and upgrades
sudo apt update
sudo apt upgrade
sudo apt autoremove

sudo apt install network-manager #AB add utility for managing networks
sudo apt install net-tools #AB add another utility for managing networks
sudo apt-get install git #AB install git, just in case it is not already installed
sudo apt install yamllint #AB a tool to check the syntax of YAML files
sudo apt install sl #AB Install sl, an alias for ls



#---------------------------------------------INSTALL INGENIUM CARTOGRAPHER REPOSITORY---------------------------------------------


mkdir -p ~/Documents/GitHub #AB Create the GitHub directory in the ~/Documents directory. If ~/Documents does not exist, the -p flag creates it also.
cd ~/Documents/GitHub

#AB Clone the ingenium_cartographer repository if it does not already exist
if ! [ -d "ingenium_cartographer" ]; then
    git clone https://github.com/JohannesByle/ingenium_cartographer.git
fi

cd ingenium_cartographer
git switch jazzy

for file in *; do #AB Iterate through all files within it
  if [[ "$file" == *.sh ]]; then #AB If the file is a bash script (i.e., if it ends in .sh)...
    chmod +x $file #AB ...then mark it as executable
  fi
done



#---------------------------------------------INSTALL ROS JAZZY AND DRIVERS---------------------------------------------


#AB Install ROS Jazzy
./Install_Jazzy.sh 

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-jazzy-velodyne #AB Install the IMU driver. It's in a stack hosted (I believe) on the ROS website.
sudo apt-get install ros-jazzy-microstrain-inertial-driver #AB Install the IMU driver. Turns out that the these drivers are now maintained as part of a built-in ROS package manager! This should make things easier for future updates.



#---------------------------------------------UPDATE THE SYSTEM AGAIN---------------------------------------------


sudo apt update
sudo apt upgrade
sudo apt autoremove



#---------------------------------------------CONFIGURE NETWORK---------------------------------------------


cd ~/Documents/GitHub/ingenium_cartographer/cartographer_config
#FK go into the config folder

sudo mv use_network_manager.yaml /etc/netplan
#FK move file that makes Ubuntu Server use NetworkManager into the correct folder

sudo chmod +x RPi_post_reboot_default_apps_installer.sh
#FK mark the second installer script as executable
sudo mv RPi_post_reboot_default_apps_installer.sh ~
#FK move second installer script to the main directory


#---------------------------------------------EXIT---------------------------------------------


cd $cwd #AB return to the original directory
echo "RPi_Default_Apps_Installer.sh has finished running now."
sleep 2
echo "System will reboot in..."
echo 5
sleep 1
echo 4
sleep 1
echo 3
sleep 1
echo 2
sleep 1
echo 1
sleep 1
reboot