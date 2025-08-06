#!/bin/bash

#FK Temporary file (until it can be integrated with or replaced by Install_LIO-SAM.sh)
#FK Finn's process of installing LIO-SAM (including setting up the docker stuff) on 8/5/2025

#FK prerequisite: docker install script
#FK prerequisite: NVIDIA toolkit install script



#---------------------------------------------INSTALL DEPENDENCIES---------------------------------------------


#FK Add humble related stuff
sudo apt install ros-humble-perception-pcl \
  	   ros-humble-pcl-msgs \
  	   ros-humble-vision-opencv \
  	   ros-humble-xacro
#FK Add GTSAM-PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev



#---------------------------------------------SET UP DIRECTORY STRUCTURE AND CLONE GITHUB---------------------------------------------


mkdir -p ~/Apps/LIO-SAM/src
cd ~/Apps/LIO-SAM/src
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd LIO-SAM
sudo git checkout ros2
cd ..


#---------------------------------------------BUILD THE ROS PACKAGE AND THE DOCKER IMAGE---------------------------------------------


sudo colcon build

#FK make the docker image
cd ~/Apps/LIO-SAM/src/LIO-SAM
sudo docker build -t liosam-humble-jammy . #FK create a docker image titled liosam-humble-jammy from a Dockerfile in the current directory



#---------------------------------------------CREATE THE DOCKER CONTAINER---------------------------------------------


#FK make the container out of the image
#FK the mount is ESPECIALLY in flux rn
cd ~
sudo docker run --init -it -d \
  --name liosam-humble-jammy-container \
  --mount type=bind,source="$(pwd)"/Documents,target=/home/Documents \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --runtime=nvidia --gpus all \
  liosam-humble-jammy \
  bash

echo "run 'sudo docker exec -it liosam-humble-jammy-container bash' to launch the container"
echo -ne "ONCE INSIDE OF CONTAINER: \n"
echo -ne "cd /home/Documents/GitHub/ingenium_cartographer \n
git config --global --add safe.directory /home/Documents/GitHub/ingenium_cartographer \n
git switch humble \n
sudo apt update \n
sudo apt upgrade \n
./process_bag.sh /home/Documents/data/test.mcap \n"
