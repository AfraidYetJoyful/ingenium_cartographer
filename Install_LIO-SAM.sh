#!/bin/bash

#AB LIO-SAM Installation Script, copied from https://github.com/TixiaoShan/LIO-SAM/tree/ros2


sudo apt update
sudo apt ugrade
sudo apt autoremove

CWD=$(pwd) 

cd ~/Apps
mkdir LIO-SAM
cd LIO-SAM

echo "Creating Dockerfile..."

echo 'FROM ros:humble

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/home/ros2_ws

# Install system dependencies (including GTSAM)
RUN apt update && apt install -y \
    git \
    wget \
    lsb-release \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-navigation2 \
    ros-humble-gtsam \
    libgoogle-glog-dev \
    libgflags-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    libboost-all-dev \
    libproj-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*



# Set up ROS 2 workspace
RUN mkdir -p /home/ros2_ws/src
WORKDIR /home/ros2_ws

# Clone the official LIO-SAM ROS2 fork
RUN git clone https://github.com/TixiaoShan/LIO-SAM.git src/LIO-SAM && \
    cd src/LIO-SAM && \
    git checkout ros2 && \
    git submodule update --init --recursive

# Build the workspace
WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Automatically source ROS setup and workspace setup on container start
RUN echo '\''source /opt/ros/humble/setup.bash'\'' >> ~/.bashrc && \
    echo '\''source /home/ros2_ws/install/setup.bash'\'' >> ~/.bashrc

# Set default command
CMD ["bash"]' > Dockerfile

echo "Building docker image..."

sudo docker build --no-cache --debug -t lio-sam-humble .

echo "LIO_SAM has finished installing."

cd $CWD


#AB to run the docker, use sudo docker run -it --rm   --net=host   --privileged   -v ~/Documents/Data:/data   lio-sam-humble


# #AB Install relevant ROS Jazzy dependencies
# sudo apt install ros-jazzy-perception-pcl
# sudo apt install ros-jazzy-pcl-msgs
# sudo apt install ros-jazzy-vision-opencv
# sudo apt install ros-jazzy-xacro

# #AB Install the Georgia Tech Smoothing And Mapping library
# # Add GTSAM-PPA
# sudo add-apt-repository ppa:borglab/gtsam-release-4.1
# sudo apt install libgtsam-dev # libgtsam-unstable-dev #AB why the unstable...? Concerning.

# #AB Install the LIO-SAM package from GitHub
# cd ~/ros2_ws/src
# git clone https://github.com/TixiaoShan/LIO-SAM.git
# cd LIO-SAM
# git checkout ros2
# cd ..
# colcon build




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