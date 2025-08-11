#!/bin/bash

#AB Bash script to reinstall all the scripts and apps necessary for the project.
#AB/FK Run on a fresh Ubuntu install with an NVIDIA GPU.
#AB NOTE: This script primarily installs a long series of "developer tools"--things necessary for the project developers (eg. CloudCompare, IDEs), but not necessarily needed on every RPi. To set up a new RPi, see Ubuntu-Core-RPi-Default-Packages-Installer.sh

RED='\033[0;31m' #AB format echo text as red
NC='\033[0m' #AB format echo text as "no color"
BOLD_CYAN='\e[1;36m'



#---------------------------------------------INSTALL BASIC PACKAGES---------------------------------------------


echo -e "\e[38;5;82mInstalling base packages...\033[0m"
sleep 1

echo -e "\e[38;5;82mUpdating apt...\033[0m"
sudo apt update
sudo apt upgrade

echo -e "\e[38;5;82mInstalling htop, openss-server, gnome-keyring, gnome-tweaks, snapd, yamllint, sl, pip, and colcon...\033[0m"
sudo apt install htop #AB Disk space monitor
sudo apt install openssh-server #AB SSH client
sudo apt-get install gnome-keyring #AB Install a secure cryptographic library needed by VS Code
sudo apt install gnome-tweaks #AB An OS customization tool
sudo apt install snapd #AB A package manager
sudo apt install yamllint #AB a tool to check the syntax of YAML files
sudo apt install sl #AB Install sl, an alias for ls
sudo apt install pip #AB Install pip, a package manager for Python

#AB Install colcon, the build manager for ROS2
echo -e "\e[38;5;82mInstalling colcon...\033[0m"
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions 

echo -e "\e[38;5;82mInstalling VS Code, Firefox, CloudCompare, and Blender...\033[0m"
sudo snap install --classic code #AB Visual Studio Code, a git-integrated IDE for basically all computer languages
sudo snap install firefox
sudo snap refresh firefox #AB Update the default-installed firefox to the latest version
sudo snap install cloudcompare #AB Install CloudCompare (a point-cloud processing software)
sudo snap install --classic blender #AB Install blender (a 3D modeling software)

echo -e "\e[38;5;82mInstalling git and configuring git and VS Code...\033[0m"
sudo apt-get install git #AB Install and then configure git (a source control software for coders)
git config --global user.email "ingenium.lidar@outlook.com"
git config --global user.name "Ingenium-LiDAR"

code --password-store="gnome-libsecret" #AB Configure VS Code to use Gnome Keyring



#---------------------------------------------INSTALL "ingenium_cartographer" REPOSITORY---------------------------------------------


echo -e "\e[38;5;82mSetting up the default directory structure...\033[0m"
mkdir -p ~/Documents/GitHub
mkdir -p ~/Documents/Data
mkdir ~/Apps


echo -e "\e[38;5;82mInstalling ingenium_cartographer repository...\033[0m"
if ! [ -d ~/Documents/GitHub/ingenium_cartographer ]; then #AB If a directory called ingenium_cartographer does not already exist in ~/Documents/GitHub...
  cd ~/Documents/GitHub #AB ...navigate to the ~/Documents/GitHub directory
  git clone https://github.com/JohannesByle/ingenium_cartographer
  cd ingenium_cartographer #AB Enter the newly cloned repository
  git switch humble #AB Switch to the jazzy branch of the ingenium_cartographer repository
fi

for file in *; do #AB Iterate through all files within it
    if [[ "$file" == *.sh ]]; then #AB If the file is a bash script (i.e., if it ends in .sh)...
      chmod +x "$file" #AB ...then mark it as executable
    elif [ -d "$file" ]; then
      for subfile in "$file"; do #AB Iterate through all files within it
        if [[ "$subfile" == *.sh ]]; then #AB If the file is a bash script (i.e., if it ends in .sh)...
          chmod +x "$subfile" #AB ...then mark it as executable
        fi
      done
    fi
done

gsettings set org.gnome.desktop.background picture-uri file:~/Documents/GitHub/ingenium_cartographer/blanchard.png #AB Set the desktop background to blanchard.png from the GitHub.



#---------------------------------------------INSTALL ROS2 Jazzy---------------------------------------------


echo -e "\e[38;5;82mInstalling ROS2 Humble...\033[0m"
cd ~/Documents/GitHub/ingenium_cartographer/agent_scripts #AB Navigate to the ingenium_cartographer/agent_scripts directory. 
./Install_Humble.sh #AB Run the Install_Jazzy.sh script to install ROS Jazzy 



#---------------------------------------------INSTALL DOCKER AND NVIDIA CONTAINER TOOLKIT---------------------------------------------


cd ~/Documents/GitHub/ingenium_cartographer/agent_scripts #AB Navigate to the ingenium_cartographer/agent_scripts directory. Technically unnecessary at this stage since the script is already there, but best to make it explicit where the program needs to be.
echo -e "\e[38;5;82mInstalling Docker and NVIDIA Container Toolkit...\033[0m"
./Install_Docker.sh #FK Run the Install_Docker.sh script to install docker
echo -e "\e[38;5;82mInstalling NVIDIA Container Toolkit...\033[0m"
./Install_NVIDIA_Docker_Tools.sh #FK Run the Install_NVIDIA_Docker_Tools.sh to install NVIDIA Container toolkit



#---------------------------------------------INSTALL LIO-SAM---------------------------------------------


echo -e "\e[38;5;82mInstalling LIO-SAM...\033[0m"
if ! [ -d ~/Apps/LIO-SAM ]; then #AB If a directory called LIO-SAM is not already in the ~/Apps directory...
  cd ~/Documents/GitHub/ingenium_cartographer/agent_scripts #AB ...navigate to the ingenium_cartographer directory
  ./Finns_Install_LIO-SAM.sh #FK temp LIO-SAM installer + docker container launch file #AB Installs inside the ~/APps directory
fi



#---------------------------------------------CLEANUP---------------------------------------------


echo -e "\e[38;5;82mCleaning up...\033[0m"
echo 'alias cleanup="./cleanup.sh"' >> ~/.bashrc #AB add the alias cleanup to the system ~/.bashrc file. It will now run ./cleanup.sh whenever the command "cleanup" is entered.
echo 'alias update="sudo apt update && sudo apt upgrade && sudo apt autoremove"' >> ~/.bashrc #AB add the alias update to the system ~/.bashrc file. It will now update, upgrade, and finally autoremove all unnecessary files whenever the command "update" is entered.

echo -ne "Running sudo apt autoremove:\n"
sudo apt autoremove #AB Remove all files not needed in the system. Frees up a variable amount of space (on the Jun 24, 2025 reinstall, I had superfluous firmware. You never know...)

echo -e "\e[38;5;82mDefault_Apps_Installer.sh has finished running now.\033[0m"
