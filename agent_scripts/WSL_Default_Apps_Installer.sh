#!/bin/bash

echo -e "\e[38;5;82mUpdating apt..."
sudo apt update
sudo apt upgrade
sudo apt autoremove

echo -e "\e[38;5;82mInstalling Gnome Desktop..."
sudo apt install ubuntu-gnome-desktop
echo -e "\e[38;5;82mInstalling Gnome Tweaks..."
sudo apt install ubuntu-gnome-tweaks

echo -e "\e[38;5;82mInstalling ingenium_cartographer repository..."
if ! [ -d ~/Documents/GitHub/ingenium_cartographer ]; then #AB If a directory called ingenium_cartographer does not already exist in ~/Documents/GitHub...
  mkdir -p ~/Documents/GitHub && cd ~/Documents/GitHub #AB then make any parts of that path that do not already exist and navigate to the GitHub directory
  git clone https://github.com/JohannesByle/ingenium_cartographer #AB Clone the ingenium_cartographer repository
  git switch humble #AB Switch to the humble branch of the ingenium_cartographer repository
fi

cd ingenium_cartographer/agent_scripts #AB Enter the newly cloned repository
chmod +x Default_Apps_Installer.sh #AB Mark DAI as executable
echo -e "\e[38;5;82mRunning Default_Apps_Installer.sh..."
./Default_Apps_Installer.sh #AB run DAI

