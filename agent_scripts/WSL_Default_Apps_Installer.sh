#!/bin/bash

echo -e "\e[38;5;82mUpdating apt...\033[0m"
sudo apt update
sudo apt upgrade
sudo apt autoremove

echo -e "\e[38;5;82mInstalling Gnome Desktop...\033[0m"
sudo apt install tasksel
sudo tasksel install ubuntu-desktop
sudo apt install gnome-tweaks

# echo -e "\e[38;5;82mInstalling ingenium_cartographer repository...\033[0m"
# if ! [ -d ~/Documents/GitHub/ingenium_cartographer ]; then #AB If a directory called ingenium_cartographer does not already exist in ~/Documents/GitHub...
#   mkdir -p ~/Documents/GitHub && cd ~/Documents/GitHub #AB then make any parts of that path that do not already exist and navigate to the GitHub directory
#   git clone https://github.com/JohannesByle/ingenium_cartographer #AB Clone the ingenium_cartographer repository
# fi

# cd ~/Documents/GitHub/ingenium_cartographer #AB Enter the newly cloned repository
# git switch humble #AB Switch to the humble branch of the ingenium_cartographer repository
# cd ~/Documents/GitHub/ingenium_cartographer/agent_scripts 

wget -O tmp_Default_Apps_Installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/humble/agent_scripts/Default_Apps_Installer.sh
chmod +x tmp_Default_Apps_Installer.sh #AB Mark DAI as executable
echo -e "\e[38;5;82mRunning Default_Apps_Installer.sh...\033[0m"
./tmp_Default_Apps_Installer.sh #AB run DAI
rm tmp_Default_Apps_Installer.sh #AB delete DAI

