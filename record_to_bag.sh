#!/bin/bash
# Run cartographer setup
# source /opt/ros/noetic/setup.bash ### Why is the source command for noetic commented?
# Start the connection to the imu (and start roscore in the background)
source ~/catkin_ws/devel_isolated/setup.bash ### Very similar commands also sourced in display_bag.sh
source ~/catkin_ws/devel/setup.bash
roslaunch ros_mscl microstrain.launch & ### The & indicates that the roslaunch of the IMU (as specified in the .launch file) may proceed while the shell moves on and executes everything else
sleep 2 ### Pause further shell execution for 2 seconds

# Start the connection to the lidar
rosrun velodyne_driver velodyne_node _model:=32C _npackets:=1 _rpm:=300 &
sleep 2

# Start recording specific nodes from the lidar and the imu
rosbag record /gx5/imu/data /velodyne_packets &
sleep 2

echo "Currently recording, press any key to exit"
read -r ### Wait for an input of any key, then proceed to the next line
rosnode kill -a ### Kills all active ROS nodes
