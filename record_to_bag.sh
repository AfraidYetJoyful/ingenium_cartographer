#!/bin/bash

#---------------------------------------------SOURCE JAZZY AND CONFIGURE ENVIRONMENT VARIABLES---------------------------------------------#


source /opt/ros/jazzy/setup.bash

ethernet=enp152s0  #AB Replace enp152s0 with the name of your ethernet port, which can be found using ip address 

#AB Configure the IP address of the ethernet port to receive data from the default IP of a VLP-32C.
sudo ip addr flush dev $ethernet
sudo ip addr add 192.168.1.100/24 dev $ethernet


#AB Publish a static transform from the base frame to the IMU frame of reference. 
#  - Indicate 0 translation (as consistent with the older files from the ROS1 version)
#  - Give a rotation quaterion (same rotation as the ROS1 system). See https://www.andre-gaschler.com/rotationconverter/ for this in Euler angles (RPY) or a rotation matrix.
#  - Specify which two frames are to be linked by this quaternion transform.
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.500001 --qy -0.499999 --qz 0.500004 --qw -0.499996 --frame-id base_link --child-frame-id imu_link &



#---------------------------------------------LAUNCH DRIVERS AS DICTATED BY ENVIRONMENT VARIABLES---------------------------------------------#


ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py & #AB Launch the velodyne driver and begin broadcasting on the /velodyne_packets topic
sleep 3

ros2 launch cartographer_config/microstrain_launch_ingenium.py & #AB Launch the IMU driver and begin broadcasting on the /imu/data topic
sleep 3



#---------------------------------------------RECORD DATA AS DICTATED BY ENVIRONMENT VARIABLES---------------------------------------------#


echo "Recording lidar and imu data..."
ros2 bag record /imu/data /velodyne_packets & #AB Record the /velodyne_packets and /imu/data topics
sleep 2



#---------------------------------------------END DATA COLLECTION AND CLEAN UP WORKSPACE---------------------------------------------#


echo "Currently recording, press any key to exit"
read -r #AB Wait for an input of any key, then proceed to cleanup

./cleanup.sh #AB This  automatically moves all directories starting with "rosbag2_" to the /Documents/Data directory, and creates that directory if it does not exist.

pkill -f ros2 && pkill -f microstrain && pkill -f launch && pkill -f rviz2 && pkill -f python3 #AB forcefully kill ALL ROS2 processes to prevent ghost proceeses from continuing.
sleep 1

echo "The program has finished running now."
exit


