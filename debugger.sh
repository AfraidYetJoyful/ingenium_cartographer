#!/bin/bash

source /opt/ros/jazzy/setup.bash

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

echo "Done."


















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