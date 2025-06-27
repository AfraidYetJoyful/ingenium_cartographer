source /opt/ros/jazzy/setup.bash

# Start the connection to the lidar
ros2 run velodyne_driver device_ip=192.168.1.201 model=32C rpm=600 &
sleep 2

# Start recording specific nodes from the lidar and the imu
ros2 bag record --all &
sleep 2

echo "Currently recording, press any key to exit"
read -r ### Wait for an input of any key, then proceed to the next line
ros2 node kill -a ### Kills all active ROS nodes