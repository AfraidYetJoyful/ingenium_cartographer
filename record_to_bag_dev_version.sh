#!/bin/bash


#---------------------------------------------SOURCE JAZZY AND CONFIGURE ENVIRONMENT VARIABLES---------------------------------------------#

source /opt/ros/jazzy/setup.bash

cwd=$(pwd)
ethernet=enp152s0
record_lidar=false
record_imu=true



#---------------------------------------------LAUNCH DRIVERS AS DICTATED BY ENVIRONMENT VARIABLES---------------------------------------------#

if [ $record_lidar = "true" ]; then #AB If record_lidar parameter is enabled...
  #AB Configure the IP address of the ethernet port to receive data from the default IP of a VLP-32C. Replace enp152s0 with the name of your ethernet port, which can be found using ip addr 
  echo "recording lidar..."
  sudo ip addr flush dev $ethernet
  sudo ip addr add 192.168.1.100/24 dev $ethernet 
  sudo ip route add 192.168.1.201 dev $ethernet
  #AB Launch the velodyne driver and begin broadcasting on the /velodyne_packets topic
  ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py &
  sleep 2
fi


if [ $record_imu = "true" ]; then #AB If record_imu parameter is enabled...
  #AB Launch the IMU driver and begin broadcasting on the /imu/data topic
 

  #ros2 launch microstrain_inertial_driver microstrain_launch_ingenium.py params_file:=$(pwd)/cartographer_config/microstrain_config.yaml &
  #ros2 launch microstrain_inertial_driver microstrain_launch.py port:=/dev/ttyACM0 baudrgnss1_enable:=falseate:=115200 imu_enable:=true filter_manual_config:=false &
  #ros2 launch microstrain_inertial_driver microstrain_launch_ingenium.py params_file:=$(pwd)/cartographer_config/microstrain_config.yaml
  #ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=/path/to/microstrain_config.yaml
  #ros2 run microstrain_inertial_driver microstrain_inertial_driver_node --ros-args --params-file $(pwd)/cartographer_config/microstrain_config.yaml &
  # Add -d flag after "launch" for debug mode
  #ros2 launch microstrain_inertial_driver microstrain_launch_ingenium.py params_file:=microstrain_config.yaml low_pass_filter_config:=false &
  #ros2 launch microstrain_inertial_driver microstrain_launch_ingenium.py params_file:=/home/lidar/Documents/GitHub/ingenium_cartographer/cartographer_config/microstrain_config.yaml low_pass_filter_config:=false &
  ros2 launch microstrain_inertial_driver microstrain_launch_ingenium.py params_file:=/home/lidar/Documents/GitHub/ingenium_cartographer/cartographer_config/microstrain_config.yaml #microstrain_inertial_driver_node.tf_mode:=0 microstrain_inertial_driver.tf_mode:=0
  # ros2 param list
  # ros2 param get /microstrain_inertial_driver /tf_mode

  sleep 2
fi


#---------------------------------------------RECORD DATA AS DICTATED BY ENVIRONMENT VARIABLES---------------------------------------------#



#AB Record different topics depending on which parameters are enabled. This is for DEV VERSION ONLY. In production version, only the first option will be allowed, and all others will throw an error (since either kind of data is useless without the other for inertial SLAM purposes)
if [ $record_lidar = "true" ] &&  [ $record_imu = "true" ]; then 
  echo "Recording lidar and imu data..."
  ros2 bag record /imu/data /velodyne_packets &
elif [ $record_lidar = "true" ] && [ $record_imu = "false" ]; then
  echo "Recording lidar data only..."
  ros2 bag record /velodyne_packets &
elif [ $record_lidar = "false" ] && [ $record_imu = "true" ]; then
  echo "Recording imu data only..."
  ros2 bag record /imu/data &
else
  echo "Error: No topics recorded"
  exit
fi



#---------------------------------------------END DATA COLLECTION AND CLEAN UP WORKSPACE---------------------------------------------#



sleep 2
echo "Currently recording, press any key to exit"
read -r #AB Wait for an input of any key, then proceed to cleanup



./cleanup.sh #AB This  automatically moves all directories starting with "rosbag2_" to the /Documents/Data directory, and creates that directory if it does not exist.

sleep 5 #AB wait a good long while to let the recording process finish
pkill -f ros2 && pkill -f microstrain && pkill -f launch && pkill -f rviz2 && pkill -f python3 #AB forcefully kill ALL ROS2 processes to prevent ghost proceeses from continuing.
echo "The program has finished running now."
exit




#---------------------------------------------OLD CODE AND CHATGPT ERROR ANALYSIS---------------------------------------------#
#AB [Delete this section from the production version]



# ros2 run velodyne_driver velodyne_node device_ip=192.168.1.201 model=32C rpm=600 &
# ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py
# ros2 run velodyne_driver velodyne_driver_node --ros-args \ &
#   -p device_ip:=192.168.1.201 \
#   -p model:=32C \



# ChatGPT said:

# Thanks for the full log — this confirms that even with filter_enable:=false, the driver is still attempting to configure low-pass filters, which is unexpected and problematic. This is very likely a bug in the ROS 2 Jazzy version of microstrain_inertial_driver, specifically how it handles filter_enable and filter_manual_config.
# 🔍 What’s Going Wrong?

# Despite setting:

# filter_enable:=false

# and possibly:

# filter_manual_config:=false

# You're still seeing:

# [microstrain_inertial_driver]: Configuring low pass filter with: ...
# [FATAL] Failed to configure node

# That shouldn't happen. This confirms that the filter_enable check is not properly respected in the driver code, and it's proceeding to configure filters on a device that doesn't support that command, leading to failure.
# ✅ Workaround: Disable Filter Config Code at Source

# To truly resolve this, you should:
# 1. ✅ Clone the source version of the driver:

# cd ~/ros2_ws/src
# git clone --branch ros2 https://github.com/LORD-MicroStrain/microstrain_inertial.git

# 2. 🧩 Edit microstrain_config.cpp

# Edit the file:

# nano ~/ros2_ws/src/microstrain_inertial/microstrain_inertial_driver/src/microstrain_config.cpp

# Find and comment out or disable this section (approx. lines 378–425):

# // These configure the low-pass filters — disable them:
# if (RCLParamUtils::getParamOrWarn(node_, "filter_manual_config", filter_manual_config_))
# {
#   // Comment out this whole block to prevent filter config
#   // configureLowPassFilter(0x04, ...);
#   // configureLowPassFilter(0x05, ...);
#   // configureLowPassFilter(0x06, ...);
# }

# 3. 🔧 Rebuild the driver:

# cd ~/ros2_ws
# colcon build --packages-select microstrain_inertial_driver
# source install/setup.bash

# 4. 🚀 Re-launch:

# Now try running your launch file again:

# ros2 launch microstrain_inertial_driver microstrain_launch.py \
#   port:=/dev/ttyACM0 \
#   baudrate:=115200 \
#   imu_enable:=true \
#   gnss1_enable:=false \
#   gnss2_enable:=false \
#   filter_enable:=false \
#   filter_manual_config:=false

# This time, the node should skip filter configuration entirely — and run successfully.
# 📝 Summary
# Issue	Fix
# Driver crashes trying to configure low-pass filters	Caused by device not supporting them
# filter_enable:=false not working	Bug in driver
# Real fix	Build from source, patch config logic to skip filter setup

# Would you like me to help you prepare a minimal patch file or give you an exact diff to apply?


#AB past YAMLs 

# microstrain_inertial_driver:
#   ros__parameters:
#     # You should change this section of config to match your setup
#     port : '/dev/ttyACM0'
#     baudrate : 115200

#     # This will cause the node to convert any NED measurements to ENU
#     # This will also cause the node to convert any vehicle frame measurements to the ROS definition of a vehicle frame
#     use_enu_frame : True

#     # Configure some frame IDs
#     frame_id : 'gx5_15_link'  # Frame ID of all of the filter messages. Represents the location of the CV7-INS in the tf tree

#     # Disable the transform from the mount to frame id transform as it will be handled in the launch file
#     publish_mount_to_frame_id_transform : False

# microstrain_inertial_driver_node:
#   ros__parameters:
#     port: /dev/ttyACM0
#     baudrate: 115200

#     imu_enable: true
#     publish_imu: true

#     gnss1_enable: false
#     gnss2_enable: false

#     filter_enable: false
#     filter_manual_config: false

#     tf_mode: 0

#     debug: true

