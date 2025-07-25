#!/bin/bash

#AB Run on a clean Ubuntu Server 24.04.2 LTS system

cwd = $(pwd)
read -p "Enter password for hotspot: " hotspot_password #AB Ask the user to enter a password for the hotspot. 


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



#---------------------------------------------CONFIGURE NETWORK---------------------------------------------


cd ~/Documents/GitHub/ingenium_cartographer/cartographer_config
sudo mv use_network_manager.yaml /etc/netplan

nmcli device wifi hotspot ifname wlan0 ssid Hotspot4 password $hotspot_password #FK tells NetworkManager to create a connection profile for a hotspot, on the network interface (aka device) with the name (ifname = interface name) wlan0, with an ssid of Hotspot 4 (so that Hotspot4 is the name that appears for people wishing to connect to it), with a certain password
#FK tells NetworkManager to create a connection profile for a hotspot, 
# on the network interface (aka device) with the name (ifname = interface name) wlan0, 
# with an ssid of Hotspot 4 (so that Hotspot4 is the name that appears for people wishing to connect to it),
# with a certain password 
nmcli connection modify id Hotspot connection.autoconnect yes
#FK tells NetworkManager to edit the hotspot’s connection profile so that it will make the hotspot automatically on startup
nmcli connection modify id Hotspot connection.autoconnect-priority 1
#FK make the autoconnect priority greater than 0 so that the hotspot takes priority over other connections
#FK at this point, after a reboot, the hotspot should automatically start on start and before login.



#---------------------------------------------EXIT---------------------------------------------


cd $cwd #AB return to the original directory
echo "RPi_Default_Apps_Installer.sh has finished running now."