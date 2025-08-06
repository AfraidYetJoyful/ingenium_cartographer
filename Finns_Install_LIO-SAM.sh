#!/bin/bash

#FK Temporary file (until it can be integrated with or replaced by Install_LIO-SAM.sh)
#FK Finn's process of installing LIO-SAM (including setting up the docker stuff) on 8/5/2025

#FK PRECONDITION: The docker install script (Install_Docker.sh) has been run.
#FK PRECONDITION: The NVIDIA toolkit install script (Install_NVIDIA_Docker_Tools.sh) has been run.

#FK Add humble related stuff
sudo apt install ros-humble-perception-pcl \
  	   ros-humble-pcl-msgs \
  	   ros-humble-vision-opencv \
  	   ros-humble-xacro
#FK Add GTSAM-PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev

#FK NOT IN THE LIO-SAM GITHUB README DIRECTIONS (ran into issues until I did this though)
mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd LIO-SAM
sudo git checkout ros2
cd ..
sudo colcon build

#FK make the docker image
cd ~/ros2_ws/src/LIO-SAM
sudo docker build -t liosam-humble-jammy . #FK create a docker image titled liosam-humble-jammy from a Dockerfile in the current directory
