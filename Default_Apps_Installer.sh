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
sudo apt install rpi-imager #AB a software for burning OSes onto SD cards for use in a Raspberry Pi
sudo apt install gnome-tweaks #AB An OS customization tool
sudo apt install snapd #AB A package manager
sudo apt install yamllint #AB a tool to check the syntax of YAML files
sudo apt install gdm-settings libglib2.0-dev-bin #AB Another OS customization tool
sudo apt install net-tools
sudo apt install python3-pip #AB Install pip, Python's package manager.
sudo apt install python3.12-venv #AB Install a package to allow creating python virtual environments
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
  git switch jazzy #AB Switch to the jazzy branch of the ingenium_cartographer repository
fi

cd ingenium_cartographer #AB Enter the newly cloned repository
for file in *; do #AB Iterate through all files within it
  if [[ "$file" == *.sh ]]; then #AB If the file is a bash script (i.e., if it ends in .sh)...
    chmod +x $file #AB ...then mark it as executable
  fi
done

gsettings set org.gnome.desktop.background picture-uri file:~/Documents/GitHub/ingenium_cartographer/blanchard.png #AB Set the desktop background to blanchard.png from the GitHub.




#---------------------------------------------INSTALL ROS2 Jazzy---------------------------------------------


echo "Installing ROS2 Jazzy Jalisco...\n"
cd ~/Documents/GitHub/ingenium_cartographer #AB Navigate to the ingenium_cartographer directory. Technically unnecessary at this stage since the script is already there, but best to make it explicit where the program needs to be.
./Install_Jazzy.sh #AB Run the Install_Jazzy.sh script to install ROS Jazzy 



#---------------------------------------------INSTALL HARDWARE DRIVERS---------------------------------------------


sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-jazzy-velodyne #AB Install the Velodyne driver. It's in a stack hosted (I believe) on the ROS website.
sudo apt-get install ros-jazzy-microstrain-inertial-driver #AB Install the IMU driver. Turns out that the these drivers are now maintained as part of a built-in ROS package manager! This should make things easier for future updates.



#---------------------------------------------CONFIGURE PORTS AND IP ADDRESSES---------------------------------------------


# sudo ip addr flush dev enp152s0 
# sudo ip addr add 192.168.1.100/24 dev enp152s0 
#AB This section rewrites your ethernet IP to be on the same network as the VLP-32C default. If your sensors are not connecting, you're probably on the wrong subnet.
sudo ip route add 192.168.1.201 dev enp152s0 #AB Replace enp152s0 with the name of your ethernet port, which can be found by running ip address 




#---------------------------------------------INSTALL VELOVIEW---------------------------------------------


echo "Installing VeloView..."
CURRENT_DIRECTORY=$(pwd) #AB store the current directory in a variable
cd ~ #AB create a new directory called "Apps" within the directory ~ (the user default) and navigate into it
mkdir Apps
cd Apps

#AB Download VeloView 5.1 for Ubuntu from the web
curl "https://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v5.9&type=app&os=Linux&downloadFile=VeloView-5.1.0-Ubuntu18.04-x86_64.tar.gz" --output veloview.tar.gz
tar -xzf veloview.tar.gz #AB Extract it from the archive. Extracts by default to a directory called VeloView-5.1.0-Ubuntu18.04-x86_64
chmod +x "VeloView-5.1.0-Ubuntu18.04-x86_64/bin/VeloView" #AB Make the VeloView binary executable
VELOVIEW_EXEC_PATH="$(pwd)/VeloView-5.1.0-Ubuntu18.04-x86_64/bin/VeloView" #AB Get the absolute path to the executable

#AB Create a Desktop entry file for all users linking to the VeloView binary
sudo bash -c "cat > '/usr/share/applications/veloview.desktop' <<EOF
[Desktop Entry]
Version=1.0
Name=VeloView
Exec=$VELOVIEW_EXEC_PATH
Icon=utilities-terminal
Terminal=false
Type=Application
Categories=Graphics;
EOF"

sudo chmod +x "/usr/share/applications/veloview.desktop" #AB Make the desktop file into an executable
rm veloview.tar.gz #AB delete the archive previously downloaded

cd $CURRENT_DIRECTORY #AB return to the directory the script was in before installing VeloView



#---------------------------------------------INSTALL LIO-SAM---------------------------------------------


if ! [ -d ~/Apps/LIO-SAM ]; then #AB If a directory called LIO-SAM is not already in the ~/Apps directory...
  cd ~/Documents/GitHub/ingenium_cartographer #AB ...navigate to the ingenium_cartographer directory
  ./Install_LIO-SAM.sh #AB Run a script to install LIO-SAM inside a docker in the ~/Apps directory
fi



#---------------------------------------------CLEANUP---------------------------------------------


echo 'alias cleanup="./cleanup.sh"' >> ~/.bashrc #AB add the alias cleanup to the system ~/.bashrc file. It will now run ./cleanup.sh whenever the command "cleanup" is entered.
echo 'alias update="sudo apt update && sudo apt upgrade && sudo apt autoremove"' >> ~/.bashrc #AB add the alias update to the system ~/.bashrc file. It will now update, upgrade, and finally autoremove all unnecessary files whenever the command "update" is entered.

echo -ne "Running sudo apt autoremove:\n"
sudo apt autoremove #AB Remove all files not needed in the system. Frees up a variable amount of space (on the Jun 24, 2025 reinstall, I had superfluous firmware. You never know...)

