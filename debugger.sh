#!/bin/bash

source /opt/ros/jazzy/setup.bash #AB Source ROS Jazzy



#AB Launch the Microstrain Inertial Driver
ros2 launch cartographer_config/microstrain_launch_ingenium.py & #params_file:=/home/lidar/Documents/GitHub/ingenium_cartographer/cartographer_config/microstrain_config.yaml &
sleep 4

#AB If the microstrain_inertial_driver_node is alive, then...
if ros2 node list | grep -w -q "microstrain_inertial_driver_node"; then
    #AB Print its parameters and their values
    echo "device_type"
    ros2 param get /microstrain_inertial_driver_node device_type
    echo "tf_mode"
    ros2 param get /microstrain_inertial_driver_node tf_mode
    echo "imu_enable"
    ros2 param get /microstrain_inertial_driver_node imu_enable
    echo "publish_imu"
    ros2 param get /microstrain_inertial_driver_node publish_imu
    echo "port"
    ros2 param get /microstrain_inertial_driver_node port
    echo "baudrate"
    ros2 param get /microstrain_inertial_driver_node baudrate
    echo "filter_heading_source"
    ros2 param get /microstrain_inertial_driver_node filter_heading_source
    echo "filter_init_position"
    ros2 param get /microstrain_inertial_driver_node filter_init_position
    echo "filter_init_velocity"
    ros2 param get /microstrain_inertial_driver_node filter_init_velocity
    echo "filter_init_attitude"
    ros2 param get /microstrain_inertial_driver_node filter_init_attitude
    echo "device_setup"
    ros2 param get /microstrain_inertial_driver_node device_setup
    echo "debug"
    ros2 param get /microstrain_inertial_driver_node debug
    echo "low_pass_filter_config"
    ros2 param get /microstrain_inertial_driver_node low_pass_filter_config
    echo "raw_file_enable"
    ros2 param get /microstrain_inertial_driver_node raw_file_enable
    echo "timestamp_source"
    ros2 param get /microstrain_inertial_driver_node timestamp_source
    echo "filter_manual_config"
    ros2 param get /microstrain_inertial_driver_node filter_manual_config
    echo "filter_auto_heading_alignment_selector"
    ros2 param get /microstrain_inertial_driver_node filter_auto_heading_alignment_selector
fi

echo "Killing *..."

pkill -f ros2 && pkill -f microstrain && pkill -f launch && pkill -f rviz2 && pkill -f python3 #AB forcefully kill ALL ROS2 processes to prevent ghost proceeses from continuing.
sleep 1
echo "The program has finished running now."
exit















# ros2 param get /microstrain_inertial_driver_node ntrip_interface_enable
# ros2 param get /microstrain_inertial_driver_node gnss1_enable
# ros2 param get /microstrain_inertial_driver_node gnss2_enable
# ros2 param get /microstrain_inertial_driver_node gnss1_port
# ros2 param get /microstrain_inertial_driver_node gnss1_baudrate
# ros2 param get /microstrain_inertial_driver_node gnss2_port
# ros2 param get /microstrain_inertial_driver_node gnss2_baudrate
# ros2 param get /microstrain_inertial_driver_node gnss1_filter_heading_source
# ros2 param get /microstrain_inertial_driver_node gnss2_filter_heading_source
# ros2 param get /microstrain_inertial_driver_node gnss1_filter_init_position
# ros2 param get /microstrain_inertial_driver_node gnss1_filter_init_velocity
# ros2 param get /microstrain_inertial_driver_node gnss1_filter_init_attitude
# ros2 param get /microstrain_inertial_driver_node gnss2_filter_init_position
# ros2 param get /microstrain_inertial_driver_node gnss2_filter_init_velocity
# ros2 param get /microstrain_inertial_driver_node gnss2_filter_init_attitude
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_interface_enable
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_interface_enable
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_mountpoint
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_mountpoint
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_server
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_server
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_port
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_port
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_username
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_username
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_password
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_password
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_baudrate
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_baudrate
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_filter_heading_source
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_filter_heading_source
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_filter_init_position
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_filter_init_velocity
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_filter_init_attitude
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_filter_init_position
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_filter_init_velocity
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_filter_init_attitude
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_enable
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_enable
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_mountpoint
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_mountpoint
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_server
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_server
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_port
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_port
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_username
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_username
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_password
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_password
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_baudrate
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_baudrate
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_filter_heading_source
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_filter_heading_source
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_filter _init_position
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_filter_init_velocity
# ros2 param get /microstrain_inertial_driver_node gnss1_ntrip_filter_init_attitude
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_filter_init_position
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_filter_init_velocity
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_filter_init_attitude
# ros2 param get /microstrain_inertial_driver_node    gnss1_ntrip_enable
# ros2 param get /microstrain_inertial_driver_node gnss2_ntrip_enable 