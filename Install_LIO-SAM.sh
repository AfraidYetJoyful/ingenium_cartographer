#!/bin/bash

#AB LIO-SAM Installation Script. Installs from https://github.com/TixiaoShan/LIO-SAM/tree/ros2



#AB Citations for LIO-SAM and LeGO-LOAM, as requested in the README.md on Github.

# @inproceedings{liosam2020shan,
#   title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},
#   author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela},
#   booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
#   pages={5135-5142},
#   year={2020},
#   organization={IEEE}
# }

# @inproceedings{legoloam2018shan,
#   title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
#   author={Shan, Tixiao and Englot, Brendan},
#   booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
#   pages={4758-4765},
#   year={2018},
#   organization={IEEE}
# }

source /opt/ros/humble/setup.bash

sudo apt update
sudo apt upgrade
sudo apt autoremove



sudo apt install ros-humble-perception-pcl \
  	   ros-humble-pcl-msgs \
  	   ros-humble-vision-opencv \
  	   ros-humble-xacro


# Add GTSAM-PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev


if ! [ -d ~/Apps/LIO-SAM/src ]; then #AB if ~/Documents/GitHub does not yet exist, then create it. 
  mkdir -p ~/Apps/LIO-SAM/src

cd ~/Apps/LIO-SAM/src
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd LIO-SAM
git checkout ros2
cd ..
colcon build


docker build -t liosam-humble-jammy .

echo "Process has finished. Press enter to exit"
read -r #AB Wait for an input of any key, then proceed to cleanup

sudo apt autoremove