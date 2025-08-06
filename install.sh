#!/bin/bash

parameter=$1 #AB take the parameter passed to the script


function print_help() { #AB This function prints the help text
    echo '
------------------------------------------------------------------------------------------HELP PAGE FOR install.sh------------------------------------------------------------------------------------------

Usage: ./install.sh ARGUMENT

A script for installing various specific software packages used by the Ingenium LiDAR team. 


ARGUMENT

    dev-jazzy           Installs the LiDAR team developer tools for ROS2 Jazzy Jalisco. Use only on Ubuntu 24.04.1 LTS Desktop
    dev-humble          Installs the LiDAR team developer tools for ROS2 Humble Hawksbill. Use only on Ubuntu 22.04.5 LTS Desktop
    rpi                 Installs tools for data acquisition ONLY. Use on Raspberry Pi 3 hardware with Ubuntu 24.04.2 LTS Server
    slam                Currently, this installs the LiDAR team developer tools for ROS2 Humble Hawksbill. Eventually, it will install only necessary tools for SLAM. Use only on Ubuntu 22.04.5 LTS Desktop
    -h, --help          Prints this help page
    sl                  ...try it and see
    [NONE]              Prints this help page


EXAMPLE
    
    To install the LiDAR team developer tools for ROS2 Jazzy Jalisco on a clean Ubuntu 24.04.1 LTS Desktop, you would run:
    ./install.sh dev-jazzy


For more details or more help with this script, please see the GitHub README.md file, located at https://github.com/JohannesByle/ingenium_cartographer/blob/jazzy/README.md
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------'
   
}



if [ $parameter == "dev-jazzy" ]; then #AB Download the Jazzy DAI
    echo "Installing Ingenium LiDAR's dev-jazzy package"
    wget -O ingenium_lidar_installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/jazzy/Default_Apps_Installer.sh

else if [ $parameter == "dev-humble" ]; then #AB Download the Humble DAI
    echo "Installing Ingenium LiDAR's dev-humble package"
    wget -O ingenium_lidar_installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/humble/Default_Apps_Installer.sh

else if [ $parameter == "rpi" ]; then #AB Download the Jazzy RDAI
    echo "Installing Ingenium LiDAR's rpi package"
    wget -O ingenium_lidar_installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/jazzy/RPi_Default_Apps_Installer.sh

else if [ $parameter == "slam" ]; then #AB Someday, may install something else. For now, download Humble DAI since it runs ILS
    echo "slam"
    wget -O ingenium_lidar_installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/humble/Default_Apps_Installer.sh

else if [ $parameter == "--help" ] || [ $parameter == "--h" ]; then; print_help #AB Print the help page
    
else if [ $parameter == "sl" ]; then
    sudo apt install sl # Install critical dependency
    echo ""
    echo "He he he..."
    sleep 2
    sl

else
    echo "Parameter not recognized. Printing help page..."
    print_help 

fi


if [ $parameter == "dev-jazzy" ] || [ $parameter == "dev-humble" ] || [ $parameter == "rpi" ] || [ $parameter == "slam" ]; then #AB if the script actually downloaded something meaningful...
    rm $0 #AB Delete install.sh
    chmod +x ingenium_lidar_installer.sh #AB Mark the downloaded script as executable
    ./ingenium_lidar_installer.sh #AB Run the downloaded script
    rm ingenium_lidar_installer.sh #AB Delete the now obsolete downloaded script
