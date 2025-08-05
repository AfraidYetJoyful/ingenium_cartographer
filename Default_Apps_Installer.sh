#!/bin/bash

#AB Bash script to reinstall all the scripts and apps necessary for the project. Run on a fresh Ubuntu install.
#AB NOTE: This script primarily installs a long series of "developer tools"--things necessary for the project developers (eg. CloudCompare, IDEs), but not necessarily needed on every RPi. To set up a new RPi, see Ubuntu-Core-RPi-Default-Packages-Installer.sh

RED='\033[0;31m' #AB format echo text as red
NC='\033[0m' #AB format echo text as "no color"



#---------------------------------------------INSTALL BASIC PACKAGES---------------------------------------------
echo -ne "Installing base packages...\n"
sleep 1

echo "Running sudo apt update and upgrade: "
sudo apt update
sudo apt upgrade
echo "Installing htop, openssh, blender, snapd, gnome-tweaks, VS Code, CloudCompare, gnome-keyring, rpi-imager, Firefox, yamllint, snapd, net-tools, gdm-toolkit, pip, Docker, and git\n"
sudo apt install htop #AB Disk space monitor
sudo apt install openssh-server #AB SSH client
sudo apt-get install gnome-keyring #AB Install a secure cryptographic library needed by VS Code
sudo apt install gnome-tweaks #AB An OS customization tool
sudo apt install snapd #AB A package manager
sudo apt install yamllint #AB a tool to check the syntax of YAML files
sudo apt install sl #AB Install sl, an alias for ls


sudo snap install --classic code #AB Visual Studio Code, a git-integrated IDE for basically all computer languages
sudo snap install firefox
sudo snap refresh firefox #AB Update the default-installed firefox to the latest version
sudo snap install cloudcompare #AB Install CloudCompare (a point-cloud processing software)
sudo snap install --classic blender #AB Install blender (a 3D modeling software)

sudo apt-get install git #AB Install and then configure git (a source control software for coders)
git config --global user.email "ingenium.lidar@outlook.com"
git config --global user.name "Ingenium-LiDAR"

code --password-store="gnome-libsecret" #AB Configure VS Code to use Gnome Keyring

#---------------------------------------------INSTALL "ingenium_cartographer" REPOSITORY---------------------------------------------


if ! [ -d ~/Documents/GitHub ]; then #AB if ~/Documents/GitHub does not yet exist, then create it. 
  mkdir -p ~/Documents/GitHub
fi
if ! [ -d ~/Documents/GitHub/ingenium_cartographer ]; then #AB If a directory called ingenium_cartographer does not already exist in ~/Documents/GitHub...
  cd ~/Documents/GitHub #AB ...navigate to the ~/Documents/GitHub directory
  git clone https://github.com/JohannesByle/ingenium_cartographer
  git switch humble #AB Switch to the jazzy branch of the ingenium_cartographer repository
fi

cd ingenium_cartographer #AB Enter the newly cloned repository
for file in *; do #AB Iterate through all files within it
  if [[ "$file" == *.sh ]]; then #AB If the file is a bash script (i.e., if it ends in .sh)...
    chmod +x $file #AB ...then mark it as executable
  fi
done

gsettings set org.gnome.desktop.background picture-uri file:~/Documents/GitHub/ingenium_cartographer/blanchard.png #AB Set the desktop background to blanchard.png from the GitHub.




#---------------------------------------------INSTALL ROS2 Jazzy---------------------------------------------


echo "Installing ROS2 Humble Hawksbill...\n"
cd ~/Documents/GitHub/ingenium_cartographer #AB Navigate to the ingenium_cartographer directory. Technically unnecessary at this stage since the script is already there, but best to make it explicit where the program needs to be.
./Install_Humble.sh #AB Run the Install_Jazzy.sh script to install ROS Jazzy 



#---------------------------------------------INSTALL LIO-SAM---------------------------------------------


# if ! [ -d ~/Apps/LIO-SAM ]; then #AB If a directory called LIO-SAM is not already in the ~/Apps directory...
#   cd ~/Documents/GitHub/ingenium_cartographer #AB ...navigate to the ingenium_cartographer directory
#   ./Install_LIO-SAM.sh #AB Run a script to install LIO-SAM inside a docker in the ~/Apps directory
# fi



#---------------------------------------------CLEANUP---------------------------------------------


echo 'alias cleanup="./cleanup.sh"' >> ~/.bashrc #AB add the alias cleanup to the system ~/.bashrc file. It will now run ./cleanup.sh whenever the command "cleanup" is entered.
echo 'alias update="sudo apt update && sudo apt upgrade && sudo apt autoremove"' >> ~/.bashrc #AB add the alias update to the system ~/.bashrc file. It will now update, upgrade, and finally autoremove all unnecessary files whenever the command "update" is entered.

echo -ne "Running sudo apt autoremove:\n"
sudo apt autoremove #AB Remove all files not needed in the system. Frees up a variable amount of space (on the Jun 24, 2025 reinstall, I had superfluous firmware. You never know...)

