#!/bin/bash

#AB Run on a clean Ubuntu Server 24.04.2 LTS system


#FK updates and upgrades
sudo apt update
sudo apt upgrade
sudo apt update --fix-missing
sudo apt upgrade --fix-missing
sudo apt-get update
sudo apt-get upgrade
sudo apt-get update --fix-missing
sudo apt-get upgrade --fix-missing

sudo apt install network-manager #AB add utility for managing networks

mkdir ~/Documents
mkdir ~/Documents/GitHub

cd ~/Documents/GitHub

git clone https://github.com/JohannesByle/ingenium_cartographer.git

cd ingenium_cartographer
git switch jazzy

