#!/bin/bash


#---------------------------------------------SOURCE JAZZY AND CONFIGURE ENVIRONMENT VARIABLES---------------------------------------------#

source /opt/ros/jazzy/setup.bash

cwd=$(pwd)
ethernet=enp152s0
record_lidar=true
record_imu=true

#AB Publish a static transform from the base frame to the IMU frame of reference. 
#AB Indicate 0 translation (as consistent with the older files from the ROS1 version)
#AB Give a rotation quaterion (same rotation as the ROS1 system). See https://www.andre-gaschler.com/rotationconverter/ for this in Euler angles (RPY) or a rotation matrix.
#AB Specify which two frames are to be linked by this quaternion transform.
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.500001 --qy -0.499999 --qz 0.500004 --qw -0.499996 --frame-id base_link --child-frame-id imu_link &
ros2 topic echo /imu/data &



#---------------------------------------------LAUNCH DRIVERS AS DICTATED BY ENVIRONMENT VARIABLES---------------------------------------------#

if [ $record_lidar = "true" ]; then #AB If record_lidar parameter is enabled...
  #AB Configure the IP address of the ethernet port to receive data from the default IP of a VLP-32C. Replace enp152s0 with the name of your ethernet port, which can be found using ip addr 
  echo "recording lidar..."
  sudo ip addr flush dev $ethernet
  sudo ip addr add 192.168.1.100/24 dev $ethernet
  # sudo ip route add 192.168.1.201 dev $ethernet
  #AB Launch the velodyne driver and begin broadcasting on the /velodyne_packets topic
  ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py &
  sleep 3
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
  #ros2 launch cartographer_config/microstrain_launch_ingenium.py params_file:=/home/lidar/Documents/GitHub/ingenium_cartographer/cartographer_config/microstrain_config.yaml & #microstrain_inertial_driver.tf_mode:=0
  # ros2 run some_package some_ros_executable --ros-args -p my_param:=value
  # ros2 param list
  # ros2 param get /microstrain_inertial_driver /tf_mode
  ros2 launch cartographer_config/microstrain_launch_ingenium.py &

  sleep 3
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

pkill -f ros2 && pkill -f microstrain && pkill -f launch && pkill -f rviz2 && pkill -f python3 #AB forcefully kill ALL ROS2 processes to prevent ghost proceeses from continuing.
sleep 1
echo "The program has finished running now."
exit




#---------------------------------------------OLD CODE---------------------------------------------#
#AB [Delete this section from the production version]



# ros2 run velodyne_driver velodyne_node device_ip=192.168.1.201 model=32C rpm=600 &
# ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py
# ros2 run velodyne_driver velodyne_driver_node --ros-args \ &
#   -p device_ip:=192.168.1.201 \
#   -p model:=32C \



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





#---------------------------------------------ROS1 EXHAUSTIVE PARAMS.YML [EDITED]---------------------------------------------#





# # Standalone example params file for GX3, GX4, GX/CX5, RQ1 and GQ7 series devices
# # Note: Feature support is device-dependent and some of the following settings may have no affect on your device.
# # Please consult your device's documentation for supported features

# # ****************************************************************** 
# # NOTE: This file is formatted to work with ROS and will not work if specified as the params_file argument in ROS2.
# #       If you want to override parameters for ROS2, start with https://github.com/LORD-MicroStrain/microstrain_inertial/blob/ros2/microstrain_inertial_driver/config/empty.yml
# # ****************************************************************** 

# # ******************************************************************
# # General Settings
# # ******************************************************************

# # port is the main port that the device will communicate over. For all devices except the GQ7, this is the only available port.
# # aux_port is only available for the GQ7 and is only needed when streaming RTCM corrections to the device from ROS, or if you want to publish NMEA sentences from this node
# port         : "/dev/ttyACM0"
# baudrate     : 115200
# aux_port     : "/dev/ttyACM1"
# aux_baudrate : 115200
# debug        : False

# # If set to true, this will configure the requested baudrate on the device for the main port and aux port if possible.
# # Note that this will be set on both USB and serial, but will only actually affect the baudrate of a serial connection.
# set_baud : False

# # Waits for a configurable amount of time until the device exists
# # If poll_max_tries is set to -1 we will poll forever until the device exists
# poll_port      : False
# poll_rate_hz   : 1.0
# poll_max_tries : 60

# # Number of times to attempt to reconnect to the device if it disconnects while running
# # If configure_after_reconnect is true, we will also reconfigure the device after we reconnect
# #
# # Note: It is strongly recommended to configure after reconnect unless device_setup is set to false
# #       as the device will likely initialize in a different state otherwise
# reconnect_attempts : 0
# configure_after_reconnect : True

# # Controls if the driver-defined setup is sent to the device
# #     false - The driver will ignore the settings below and use the device's current settings
# #     true  - Overwrite the current device settings with those listed below
# device_setup : True

# # Controls if the driver-defined settings are saved
# #     false - Do not save the settings
# #     true  - Save the settings in the device's non-volatile memory
# save_settings : False

# # Controls if the driver creates a raw binary file
# #     false - Do not create the file
# #     true  - Create the file
# #
# # Note: The filename will have the following format -
# #       model_number "_" serial_number "_" datetime (year_month_day_hour_minute_sec) ".bin"
# #           example: "3DM-GX5-45_6251.00001_20_12_01_01_01_01.bin"
# # Note: This file is useful for getting support from the manufacturer
# raw_file_enable : False

# # The directory to store the raw data file
# raw_file_directory : "/home/your_name"

# # Timestamp configuration
# #     0 - ROS time that the packet was received. This is the simplest, but also least accurate timestamp solution
# #     1 - GPS time. This will stamp the messages with the exact timestamps produced by the device.
# #         This is the most accurate solution, but will also produce timestamps starting at GPS time 0 until a valid GPS time is received,
# #         at which point the time will jump to the current UTC time. Useful for those doing their own timestamp processing
# #     2 - Hybrid. This will combine the two above methods to produce valid UTC times where the dt is very close to the device time dt.
# #         This is the easiest and most accurate solution for users looking to consume messages with no additional processing.
# timestamp_source : 2

# # ****************************************************************** 
# # Frame ID Settings 
# # ****************************************************************** 

# # The mode in which we will publish transforms to the below frame IDs
# #     0 - No transforms will be published between any of the non static frame ids. (if publish_mount_to_frame_id_transform is true, it will still be published, and so will the antenna and odometer transforms)
# #     1 - Global mode:
# #             Transform will be published from earth_frame_id to target_frame_id containing global position
# #     2 - Relative mode:
# #             Note: In order to use relative mode, you must configure filter_relative_position
# #             Transform will be published from earth_frame_id to map_frame_id using relative position configuration
# #             Transform between map_frame_id and target_frame_id will be published using position information reported by the device
# # for more information, see: https://wiki.ros.org/microstrain_inertial_driver/transforms
# tf_mode : 0

# # Frame ID that most header.frame_id fields will be populated with.
# frame_id: "imu_link"

# # Frame IDs determining the transforms published by this node to aid in navigation. See https://www.ros.org/reps/rep-0105.html
# # Note: If use_enu_frame is false, these frames (with the exception of earth_frame_id) will automatically have "_ned" appended to them.
# mount_frame_id     : "base_link"  # Frame ID that the device is mounted on.
# map_frame_id       : "map"
# earth_frame_id     : "earth"
# gnss1_frame_id     : "gnss_1_antenna_link"
# gnss2_frame_id     : "gnss_2_antenna_link"
# odometer_frame_id  : "odometer_link"

# # Target frame ID to publish transform to. Note that there needs to be some path of transforms between this and frame_id
# #     If tf_mode is set to 1, a transform between earth_frame_id and target_frame_id will be published
# #     If tf_mode is set to 2, a transform between map_frame_id and target_frame_id will be published
# target_frame_id : "base_link"

# # Static transform between mount_frame_id and frame_id.
# # Note: It is recommended to define this in a urdf file if you are building a robot, or are able to modify the robot's description.
# publish_mount_to_frame_id_transform  : True
# mount_to_frame_id_transform          : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] # [ x, y, z, qi, qj, qk, qw ]

# # Controls if the driver outputs data with-respect-to ENU frame
# #     false - position, velocity, and orientation are WRT the NED frame (native device frame)
# #     true  - position, velocity, and orientation are WRT the ENU frame
# #
# # Note: It is strongly recommended to leave this as True if you plan to integrate with other ROS tools
# # Note: If this is set to false, all "*_frame_id" variables will have "_ned" appended to them by the driver.
# # For more information, see: https://wiki.ros.org/microstrain_inertial_driver/use_enu_frame
# use_enu_frame : True


# # Sensor2vehicle frame transformation selector
# #     0 = None
# #     1 = Euler Angles
# #     2 = matrix
# #     3 = quaternion
# # Note: These are different ways of setting the same parameter in the device.
# #       The different options are provided as a convenience.
# #       Support for matrix and quaternion options is firmware version dependent (GQ7 supports Quaternion as of firmware 1.0.07)
# #       Quaternion order is [i, j, k, w]
# # Note: This can cause strange behavior when also using the ROS transform tree.
# #       It is recommended to not use this if you want to use the ROS transform tree unless you really know what you are doing
# filter_sensor2vehicle_frame_selector : 1
# filter_sensor2vehicle_frame_transformation_euler      : [0.0, 0.0, 0.0]
# filter_sensor2vehicle_frame_transformation_matrix     : [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# filter_sensor2vehicle_frame_transformation_quaternion : [0.0, 0.0, 0.0, 1.0]

# # Controls if the Kalman filter will auto-init or requires manual initialization
# filter_auto_init : True

# # Controls if the Kalman filter is reset after the settings are configured
# filter_reset_after_config : True


# # Controls what kind of linear acceleration data is used in the Filter IMU message.
# #     If this is set to true, the acceleration will not factor out gravity, if set to false gravity will be filtered out of the linear acceleration.
# filter_use_compensated_accel : True


# # ****************************************************************** 
# # IMU Settings 
# # ****************************************************************** 

# # Static IMU message covariance values (the device does not generate these) 
# imu_orientation_cov   : [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
# imu_linear_cov        : [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
# imu_angular_cov       : [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
# imu_mag_cov           : [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
# imu_pressure_variance : 0.01

# # ****************************************************************** 
# # Other Device Settings
# # ****************************************************************** 

# # Low pass filter configuration.
# # Note: The device may fail if one of these is enabled, and the device does not support the relevant measurement
# #   accel_* will affect data in /imu/data_raw if enabled
# #   gyro_* will affect data in /imu/data_raw if enabled
# #   mag_* will affect data in /imu/mag if enabled
# #   pressure_* will affect data in /imu/pressure if enabled
# low_pass_filter_config : True

# accel_low_pass_filter_enable    : False
# accel_low_pass_filter_auto      : False
# accel_low_pass_filter_frequency : 1

# gyro_low_pass_filter_enable    : False
# gyro_low_pass_filter_auto      : False
# gyro_low_pass_filter_frequency : 1

# mag_low_pass_filter_enable    : False
# mag_low_pass_filter_auto      : False
# mag_low_pass_filter_frequency : 1

# pressure_low_pass_filter_enable    : False
# pressure_low_pass_filter_auto      : False
# pressure_low_pass_filter_frequency : 1

# # ****************************************************************** 
# # Publisher Settings
# # ****************************************************************** 
# #
# #  Note: If set to 0, the data will not be stremaed from the device, and the publisher will not be created

# # The speed at which the individual IMU publishers will publish at.
# imu_data_raw_rate         : 1    # Rate of imu/data_raw topic
# imu_data_rate             : 100  # Rate of imu/data topic
# imu_mag_data_rate         : 0    # Rate of imu/mag topic
# imu_pressure_data_rate    : 0    # Rate of imu/pressure topic
# imu_wheel_speed_data_rate : 0    # Rate of imu/wheel_Speed topic


# # The speed at which the individual Filter publishers will publish at.
# filter_human_readable_status_data_rate : 1  # Rate of ekf/status
# filter_imu_data_rate                   : 0  # Rate of ekf/imu/data topic
#                                             # Note: Both filter_odometry_earth_data_rate and filter_odometry_map_data_rate depend on the contents of this message.
#                                             #       If either are set to a higher value, this message will be published at that rate.
# filter_llh_position_data_rate          : 0  # Rate of ekf/llh_position topic
# filter_velocity_data_rate              : 0  # Rate of ekf/velocity topic
#                                             # Note: filter_odometry_map_data_rate depends on the contents of this message.
#                                             #       If either are set to a higher value, this message will be published at that rate.
# filter_velocity_ecef_data_rate         : 0  # Rate of ekf/velocity_ecef topic
#                                             # Note: filter_odometry_earth_data_rate depends on the contents of this message.
#                                             #       If either are set to a higher value, this message will be published at that rate.
# filter_odometry_earth_data_rate        : 25 # Rate of ekf/odometry_earth topic
# filter_odometry_map_data_rate          : 25 # Rate of ekf/odometry_map topic
# filter_dual_antenna_heading_data_rate  : 0  # Rate of ekf/dual_antenna_heading topic
#                                             # Note: mip_filter_gnss_position_aiding_status_data_rate depends on the contents of this message.
#                                             #       If either are set to a higher value, this message will be published at that rate.
