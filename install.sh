#!/bin/bash

variant=$1

if [ $variant == "dev-jazzy" ]; then
    echo "Installing Ingenium LiDAR's dev-jazzy package"
    wget -O installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/jazzy/Default_Apps_Installer.sh

else if [ $variant == "dev-humble" ]; then
    echo "Installing Ingenium LiDAR's dev-humble package"
    wget -O installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/humble/Default_Apps_Installer.sh

else if [ $variant == "rpi" ]; then
    echo "Installing Ingenium LiDAR's rpi package"
    wget -O installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/jazzy/RPi_Default_Apps_Installer.sh

else if [ $variant == "slam" ]; then
    echo "slam"
    wget -O installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/humble/Default_Apps_Installer.sh
fi

chmod +x installer.sh
./installer.sh
rm installer.sh
rm $0