#!/bin/bash

sudo apt update
sudo apt upgrade
sudo apt autoremove

sudo apt install ubuntu-gnome-desktop
sudo apt install ubuntu-gnome-tweaks

if ! [ -d ~/Documents/GitHub/ingenium_cartographer ]; then #AB If a directory called ingenium_cartographer does not already exist in ~/Documents/GitHub...
  mkdir -p ~/Documents/GitHub && cd ~/Documents/GitHub #AB then make any parts of that path that do not already exist and navigate to the GitHub directory
  git clone https://github.com/JohannesByle/ingenium_cartographer #AB Clone the ingenium_cartographer repository
  git switch humble #AB Switch to the humble branch of the ingenium_cartographer repository
fi

cd ingenium_cartographer #AB Enter the newly cloned repository
chmod +x Default_Apps_Installer.sh #AB Mark DAI as executable
./Default_Apps_Installer.sh #AB run DAI

echo 'Deleting $(basename "$0")' && rm $(basename "$0") #AB Move the currently running script to Trash, since a copy of it is now present in ingenium_cartographer.