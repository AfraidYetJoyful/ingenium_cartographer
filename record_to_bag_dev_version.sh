source /opt/ros/jazzy/setup.bash

cwd=$(pwd)
ethernet=enp152s0

#AB Configure the IP address of the ethernet port to recieve data from the default IP of a VLP-32C. Replace enp152s0 with the name of your ethernet port, which can be found using ip addr 
sudo ip addr flush dev $ethernet
sudo ip addr add 192.168.1.100/24 dev $ethernet 
sudo ip route add 192.168.1.201 dev $ethernet


# Start the connection to the lidar
# ros2 run velodyne_driver velodyne_node device_ip=192.168.1.201 model=32C rpm=600 &
# ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py
# ros2 run velodyne_driver velodyne_driver_node --ros-args \ &
#   -p device_ip:=192.168.1.201 \
#   -p model:=32C \
ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py &



sleep 2

# Start recording specific nodes from the lidar and the imu
ros2 bag record --all &
sleep 2

echo "Currently recording, press any key to exit"
read -r ### Wait for an input of any key, then proceed to the next line
ros2 node kill -a ### Kills all active ROS nodes

for dir in "$(pwd)"/*/     #AB Iterate through every directory in the current working directory (i.e., the directory in which the script is running)
do
    if [[ "$dir" =~ rosbag2_* ]]; then #AB Use a regular expression to determien if any of the directories starts with the phrase "rosbag2_" and has other characters after it
        mv "$dir" "/home/lidar/Documents/Data" #AB If it matches this pattern, move it to the Data folder out of the script folder
    fi

done
