#AB Custom-configured almost-minimal launch file for 3DM-GX5-15/3DM-GX5-AR IMU, specifically for use by the Ingenium LiDAR team. 

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    print("-------------------------------------1-------------------------------------")
    # Declare Launch Arguments

    # --- Device Configuration ---
    device_type_arg = DeclareLaunchArgument('device_type', default_value=TextSubstitution(text='gx5_15'), description='The Microstrain device type (e.g., gx5_15, 3dm_gq7).')
    port_arg = DeclareLaunchArgument('port', default_value=TextSubstitution(text='/dev/ttyACM0'), description='The serial port connected to the Microstrain IMU.')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='115200', description='The baud rate for the serial connection.')

    # --- IMU and Filter Settings ---
    imu_enable_arg = DeclareLaunchArgument('imu_enable', default_value='true', description='Enable IMU data acquisition.')
    publish_imu_arg = DeclareLaunchArgument('publish_imu', default_value='true', description='Publish IMU data to ROS topics.')
    filter_manual_config_arg = DeclareLaunchArgument('filter_manual_config', default_value='true', description='Manually configure the filter settings.')
    low_pass_filter_config_arg = DeclareLaunchArgument('low_pass_filter_config', default_value='false', description='Enable low-pass filter configuration.')
    timestamp_source_arg = DeclareLaunchArgument('timestamp_source', default_value='0', description='Timestamp source for the IMU data.')
    filter_auto_heading_alignment_selector_arg = DeclareLaunchArgument('filter_auto_heading_alignment_selector', default_value='5', description='Selector for automatic heading alignment.')
    tf_mode_arg = DeclareLaunchArgument('tf_mode', default_value='0', description='Transformation frame mode.')
    filter_heading_source_arg = DeclareLaunchArgument('filter_heading_source', default_value='0', description='Source for the filter heading.')

    # --- Initial Filter State ---
    filter_init_position_x_arg = DeclareLaunchArgument('filter_init_position_x', default_value='0.0', description='Initial X position (meters).')
    filter_init_position_y_arg = DeclareLaunchArgument('filter_init_position_y', default_value='0.0', description='Initial Y position (meters).')
    filter_init_position_z_arg = DeclareLaunchArgument('filter_init_position_z', default_value='0.0', description='Initial Z position (meters).')
    filter_init_velocity_x_arg = DeclareLaunchArgument('filter_init_velocity_x', default_value='0.0', description='Initial X velocity (m/s).')
    filter_init_velocity_y_arg = DeclareLaunchArgument('filter_init_velocity_y', default_value='0.0', description='Initial Y velocity (m/s).')
    filter_init_velocity_z_arg = DeclareLaunchArgument('filter_init_velocity_z', default_value='0.0', description='Initial Z velocity (m/s).')
    filter_init_attitude_roll_arg = DeclareLaunchArgument('filter_init_attitude_roll', default_value='4.712', description='Initial roll attitude (radians).')
    filter_init_attitude_pitch_arg = DeclareLaunchArgument('filter_init_attitude_pitch', default_value='0.0', description='Initial pitch attitude (radians).')
    filter_init_attitude_yaw_arg = DeclareLaunchArgument('filter_init_attitude_yaw', default_value='1.5707', description='Initial yaw attitude (radians).')

    print("-------------------------------------2-------------------------------------")

    # Get Launch Configurations
    device_setup = True
    device_type = LaunchConfiguration('device_type')
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    imu_enable = LaunchConfiguration('imu_enable')
    publish_imu = LaunchConfiguration('publish_imu')
    filter_manual_config = LaunchConfiguration('filter_manual_config')
    low_pass_filter_config = LaunchConfiguration('low_pass_filter_config')
    timestamp_source = LaunchConfiguration('timestamp_source')
    filter_auto_heading_alignment_selector = LaunchConfiguration('filter_auto_heading_alignment_selector')
    tf_mode = LaunchConfiguration('tf_mode')
    filter_heading_source = LaunchConfiguration('filter_heading_source')

    filter_init_position = [
        LaunchConfiguration('filter_init_position_x'),
        LaunchConfiguration('filter_init_position_y'),
        LaunchConfiguration('filter_init_position_z')
    ]

    filter_init_velocity = [
        LaunchConfiguration('filter_init_velocity_x'),
        LaunchConfiguration('filter_init_velocity_y'),
        LaunchConfiguration('filter_init_velocity_z')
    ]

    filter_init_attitude = [
        LaunchConfiguration('filter_init_attitude_roll'),
        LaunchConfiguration('filter_init_attitude_pitch'),
        LaunchConfiguration('filter_init_attitude_yaw')
    ]

    print("-------------------------------------3-------------------------------------")

    # Create the Microstrain Inertial Driver Node
    microstrain_node = Node(
        package='microstrain_inertial_driver',
        executable='microstrain_inertial_driver_node',
        name='microstrain_inertial_driver_node',
        namespace='', # Root namespace
        parameters=[{
            "device_setup": device_setup,
            "imu_enable": imu_enable,
            "publish_imu": publish_imu,
            "filter_manual_config": filter_manual_config,
            "low_pass_filter_config": low_pass_filter_config,
            "device_type": device_type,
            "port": port,
            "baudrate": baudrate,
            "timestamp_source": timestamp_source,
            "filter_auto_heading_alignment_selector": filter_auto_heading_alignment_selector,
            "tf_mode": tf_mode,
            "filter_heading_source": filter_heading_source,
            "filter_init_position": filter_init_position,
            "filter_init_velocity": filter_init_velocity,
            "filter_init_attitude": filter_init_attitude,
        }]
        #output='screen'
    )

    print("Launch.py initialized successfully.")

    return LaunchDescription([
        # Add all declared arguments to the LaunchDescription
        device_type_arg,
        port_arg,
        baudrate_arg,
        imu_enable_arg,
        publish_imu_arg,
        filter_manual_config_arg,
        low_pass_filter_config_arg,
        timestamp_source_arg,
        filter_auto_heading_alignment_selector_arg,
        tf_mode_arg,
        filter_heading_source_arg,
        filter_init_position_x_arg,
        filter_init_position_y_arg,
        filter_init_position_z_arg,
        filter_init_velocity_x_arg,
        filter_init_velocity_y_arg,
        filter_init_velocity_z_arg,
        filter_init_attitude_roll_arg,
        filter_init_attitude_pitch_arg,
        filter_init_attitude_yaw_arg,
        microstrain_node
    ])





















# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#   launch_description = LaunchDescription(
#     [
#       Node( #AB Create a "Node" object to launch the Microstrain Inertial Driver
#         package    = 'microstrain_inertial_driver',
#         executable = "microstrain_inertial_driver_node", #AB Name of the executable to run
#         name       = "microstrain_inertial_driver_node", #AB Name of the node
#         namespace  = '', #AB Namespace to use when launching the nodes in this launch file (empty string for root namespace)
#         parameters = [
#           {
#               "device_setup"                           : True,
#               "imu_enable"                             : True,
#               "publish_imu"                            : True,
#               "filter_manual_config"                   : True,
#               "low_pass_filter_config"                 : False,
#               #"raw_file_enable"                        : False,
#               "device_type"                            : "gx5_15", #AB Device type to use, in this case the GX5-15 IMU
#               "port"                                   : "/dev/ttyACM0", #AB Serial port for the Microstrain IMU
#               "baudrate"                               : 115200, #AB Baud rate for the serial connection
#               "timestamp_source"                       : 0,
#               "filter_auto_heading_alignment_selector" : 5,
#               "tf_mode"                                : 0,
#               "filter_heading_source"                  : 0,
#               "filter_init_position"                   : [0.0, 0.0, 0.0], #AB Initial position in meters (x, y, z)
#               "filter_init_velocity"                   : [0.0, 0.0, 0.0], #AB Initial velocity in meters per second (vx, vy, vz)
#               "filter_init_attitude"                   : [4.712, 0.0, 1.5707], #AB Initial attitude in radians (roll, pitch, yaw)
#           } #AB Close dict of parameters
#         ] #AB Close list called "parameters"
#       ) #AB End of declaration of the Node object
#     ] #AB End of list which contains the Node object
#   ) #AB End of declaration of the LaunchDescription object, which is returned.
#   print("Launch.py initialized successfully.")
#   return launch_description 










# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, TextSubstitution
# from launch_ros.actions import Node

# def generate_launch_description():

#     # 1. Declare Launch Arguments
#     # Example 1: Simple string argument with a default value
#     robot_name_arg = DeclareLaunchArgument(
#         'robot_name',
#         default_value=TextSubstitution(text='my_robot'),
#         description='Name of the robot.'
#     )

#     # Example 2: Integer argument with a default value
#     update_rate_arg = DeclareLaunchArgument(
#         'update_rate',
#         default_value='10', # Default values for numbers are often strings
#         description='Update rate for the node in Hz.'
#     )

#     # Example 3: Boolean argument
#     use_sim_time_arg = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='false',
#         description='Use simulation time if true.'
#     )

#     # Example 4: Argument with no default (must be provided)
#     config_file_arg = DeclareLaunchArgument(
#         'config_file',
#         description='Path to the configuration file.'
#     )

#     # 2. Use the Launch Configurations
#     # Access the values of the declared arguments
#     robot_name = LaunchConfiguration('robot_name')
#     update_rate = LaunchConfiguration('update_rate')
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     config_file = LaunchConfiguration('config_file')

#     # 3. Define Nodes
#     # Pass the parameters to your nodes
#     my_node = Node(
#         package='my_package',
#         executable='my_executable',
#         name=robot_name, # Using a launch argument for the node name
#         parameters=[
#             {'update_rate': update_rate}, # Using a launch argument for a node parameter
#             {'use_sim_time': use_sim_time},
#             os.path.join(
#                 get_package_share_directory('my_package'),
#                 'config',
#                 config_file # Using a launch argument for a config file path
#             )
#         ],
#         output='screen'
#     )

#     return LaunchDescription([
#         robot_name_arg,
#         update_rate_arg,
#         use_sim_time_arg,
#         config_file_arg,
#         my_node
#     ])













































































#---------------------------------------------OLD CODE---------------------------------------------#






  # launch_description.append(microstrain_node)
  # return LaunchDescription(launch_description)
 
 
            
          #}
        #}
            # {"ntrip_interface_enable" : False}, #AB NTRIP interface is not enabled in this configuration
            # {"gnss1_enable" : False}, #AB GNSS1 is not enabled in this configuration
            # {"gnss2_enable" : False}, #AB GNSS2 is not enabled in this configuration
            # {"publish_imu" : True}, #AB IMU data will be published
            # {"device_setup" : True}, #AB Device setup is enabled  
            # AB If you uncomment these lines or add new ones, be sure to remember the commas at the end of each line.










# import os

# import ament_index_python

# from launch import LaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import IncludeLaunchDescription, ExecuteProcess
# from launch_ros.actions import Node, SetRemap

# # Path to the launch files and directories that we will use
# _MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
# _GX5_15_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'gx5_15', 'gx5_15.yml')
# _RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'gx5_15', 'display.rviz')

# def generate_launch_description():
#   return LaunchDescription([
#     # Microstrain node
#     IncludeLaunchDescription(
#       PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
#       launch_arguments={
#         'configure': 'true',
#         'activate': 'true',
#         'params_file': _GX5_15_PARAMS_FILE,
#         'namespace': '/',
#       }.items()
#     ),

#     # In this example we have no way to publish an actual map transform, so just publish a static one so we can display data on rviz
#     # If integrating into an existing system, this should be replaced with a navigation solution
#     Node(
#       package='tf2_ros',
#       executable='static_transform_publisher',
#       output='screen',
#       arguments=[
#           "--x", "0",
#           "--y", "0",
#           "--z", "100",
#           "--roll", "0",
#           "--pitch", "0",
#           "--yaw", "0",
#           "--frame-id", "map",
#           "--child-frame-id", "base_link"
#         ]
#     ),

#     # Publish a static transform for where the GX5-15 is mounted on base_link.
#     # Unless the GX5-15 is mounted exactly at base_link, you should change this to be accurate to your setup
#     Node(
#       package='tf2_ros',
#       executable='static_transform_publisher',
#       output='screen',
#       arguments=[
#           "--x", "0",
#           "--y", "0",
#           "--z", "0",
#           "--roll", "0",
#           "--pitch", "0",
#           "--yaw", "0",
#           "--frame-id", "base_link",
#           "--child-frame-id", "gx5_15_link"
#         ]
#     ),

#     # Run rviz to view the state of the application
#     Node(
#       package='rviz2',
#       executable='rviz2',
#       output='screen',
#       arguments=[
#         '-d', _RVIZ_DISPLAY_FILE
#       ]
#     ),
#   ])






















# # Software License Agreement (BSD)
# #
# # @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# # @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
# #
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions are met:
# # * Redistributions of source code must retain the above copyright notice,
# #   this list of conditions and the following disclaimer.
# # * Redistributions in binary form must reproduce the above copyright notice,
# #   this list of conditions and the following disclaimer in the documentation
# #   and/or other materials provided with the distribution.
# # * Neither the name of Clearpath Robotics nor the names of its contributors
# #   may be used to endorse or promote products derived from this software
# #   without specific prior written permission.
# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# # POSSIBILITY OF SUCH DAMAGE.
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

# from launch_ros.actions import SetRemap
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     pkg_microstrain_inertial_driver = FindPackageShare('microstrain_inertial_driver')

#     parameters = LaunchConfiguration('parameters')
#     namespace = LaunchConfiguration('namespace')
#     robot_namespace = LaunchConfiguration('robot_namespace')

#     launch_microstrain_imu = PathJoinSubstitution([
#         pkg_microstrain_inertial_driver, 'launch', 'microstrain_launch.py'])

#     arg_namespace = DeclareLaunchArgument(
#         'namespace',
#         default_value='')

#     arg_robot_namespace = DeclareLaunchArgument(
#         'robot_namespace',
#         default_value='')

#     arg_parameters = DeclareLaunchArgument(
#         'parameters',
#         default_value=PathJoinSubstitution([
#           FindPackageShare('clearpath_sensors'),
#           'config',
#           'microstrain_imu.yaml'
#         ]))

#     launch_microstrain_imu = GroupAction([
#         SetRemap('imu/data', 'data'),
#         SetRemap('/moving_ang', 'moving_ang'),
#         SetRemap('/tf', 'tf'),
#         SetRemap('/tf_static', PathJoinSubstitution(['/', robot_namespace, 'tf_static'])),

#         IncludeLaunchDescription(
#           PythonLaunchDescriptionSource([launch_microstrain_imu]),
#           launch_arguments=[
#             ('namespace', namespace),
#             ('params_file', parameters),
#             ('configure', 'true'),
#             ('activate', 'true')
#           ]
#         )
#     ])

#     ld = LaunchDescription()
#     ld.add_action(arg_namespace)
#     ld.add_action(arg_robot_namespace)
#     ld.add_action(arg_parameters)
#     ld.add_action(launch_microstrain_imu)
#     return ld











