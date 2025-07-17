# Standalone example launch file for GX3, GX4, GX/CX5, RQ1 and GQ7 series devices
# Note: Feature support is device-dependent and some of the following settings may have no affect on your device.
# Please consult your device's documentation for supported features

# import os
# import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# from lifecycle_msgs.msg import Transition
# from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_driver'
# _DEFAULT_PARAMS_FILE = os.path.join(
#   get_package_share_directory(_PACKAGE_NAME),
#   'microstrain_inertial_driver_common',
#   'config',
#   'params.yml'
# )
_DEFAULT_PARAMS_FILE = 'microstrain_config.yaml'
# _EMPTY_PARAMS_FILE = os.path.join(
#   get_package_share_directory(_PACKAGE_NAME),
#   'config',
#   'empty.yml'
# )

def generate_launch_description():

  # Declare arguments with default values
  launch_description = []
  launch_description.append(DeclareLaunchArgument('namespace',   default_value='/',                description='Namespace to use when launching the nodes in this launch file'))
  launch_description.append(DeclareLaunchArgument('node_name',   default_value=_PACKAGE_NAME,      description='Name to give the Microstrain Inertial Driver node'))
  launch_description.append(DeclareLaunchArgument('debug',       default_value='false',            description='Whether or not to log debug information.'))
  launch_description.append(DeclareLaunchArgument('params_file', default_value=_DEFAULT_PARAMS_FILE, description='Path to file that will load additional parameters'))

  # Pass an environment variable to the node to determine if it is in debug or not
  launch_description.append(SetEnvironmentVariable('MICROSTRAIN_INERTIAL_DEBUG', value=LaunchConfiguration('debug')))

  # ****************************************************************** 
  # Microstrain sensor node 
  # ****************************************************************** 
  microstrain_node = Node(
    package    = _PACKAGE_NAME,
    executable = "microstrain_inertial_driver_node",
    name       = LaunchConfiguration('node_name'),
    namespace  = LaunchConfiguration('namespace'),
    parameters = [LaunchConfiguration('params_file')],
    #   # Load the default params file manually, since this is a ROS params file, we will need to load the file manually
    #   yaml.safe_load(open(_DEFAULT_PARAMS_FILE, 'r')),

    #   # If you want to override any settings in the params.yml file, make a new yaml file, and set the value via the params_file arg
    #   LaunchConfiguration('params_file'),

    #   # Supported overrides
    #   {
    #     "debug" : LaunchConfiguration('debug')
    #   },
    # ]
  )

  launch_description.append(microstrain_node)
  return LaunchDescription(launch_description)
 
 





















































# # Standalone example launch file for GX3, GX4, GX/CX5, RQ1 and GQ7 series devices
# # Note: Feature support is device-dependent and some of the following settings may have no affect on your device.
# # Please consult your device's documentation for supported features

# import os
# import yaml
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# from lifecycle_msgs.msg import Transition
# from ament_index_python.packages import get_package_share_directory

# _PACKAGE_NAME = 'microstrain_inertial_driver'
# _DEFAULT_PARAMS_FILE = "/home/lidar/Documents/GitHub/ingenium_cartographer/cartographer_config/microstrain_config.yaml"
# # _DEFAULT_PARAMS_FILE = os.path.join(
# #   get_package_share_directory(_PACKAGE_NAME),
# #   'microstrain_inertial_driver_common',
# #   'config',
# #   'params.yml'
# # )
# # _EMPTY_PARAMS_FILE = os.path.join(
# #   get_package_share_directory(_PACKAGE_NAME),
# #   'config',
# #   'empty.yml'
# # )

# def generate_launch_description():

#   # Declare arguments with default values
#   launch_description = []
#   launch_description.append(DeclareLaunchArgument('namespace',   default_value='/',                description='Namespace to use when launching the nodes in this launch file'))
#   launch_description.append(DeclareLaunchArgument('node_name',   default_value=_PACKAGE_NAME,      description='Name to give the Microstrain Inertial Driver node'))
#   launch_description.append(DeclareLaunchArgument('debug',       default_value='false',            description='Whether or not to log debug information.'))
#   launch_description.append(DeclareLaunchArgument('params_file', default_value="/home/lidar/Documents/GitHub/ingenium_cartographer/cartographer_config/microstrain_config.yaml", description='<!> DANGEROUS <!> Absolute path to file that will load additional parameters'))

#   # Pass an environment variable to the node to determine if it is in debug or not
#   launch_description.append(SetEnvironmentVariable('MICROSTRAIN_INERTIAL_DEBUG', value=LaunchConfiguration('debug')))

#   # ****************************************************************** 
#   # Microstrain sensor node 
#   # ****************************************************************** 
#   microstrain_node = Node(
#     package    = _PACKAGE_NAME,
#     executable = "microstrain_inertial_driver_node",
#     name       = LaunchConfiguration('node_name'),
#     namespace  = LaunchConfiguration('namespace'),
#     parameters = [
#       # Load the default params file manually, since this is a ROS params file, we will need to load the file manually
#       yaml.safe_load(open(_DEFAULT_PARAMS_FILE, 'r')),

#       # If you want to override any settings in the params.yml file, make a new yaml file, and set the value via the params_file arg
#       LaunchConfiguration("/home/lidar/Documents/GitHub/ingenium_cartographer/cartographer_config/microstrain_config.yaml"),

#       # Supported overrides
#       {
#         "debug" : LaunchConfiguration('debug')
#       },
#     ]
#   )

#   launch_description.append(microstrain_node)
#   return LaunchDescription(launch_description)
  

 
 






























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



















# import os
# import yaml
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# from ament_index_python.packages import get_package_share_directory

# _PACKAGE_NAME = 'microstrain_inertial_driver'
# _DEFAULT_PARAMS_FILE = os.path.join(
#     get_package_share_directory(_PACKAGE_NAME),
#     'microstrain_inertial_driver_common',
#     'config',
#     'params.yml'
# )

# def load_yaml(path):
#     try:
#         with open(path, 'r') as f:
#             return yaml.safe_load(f)
#     except Exception as e:
#         print(f"Failed to load YAML config: {e}")
#         return {}

# def launch_setup(context, *args, **kwargs):
#     namespace = LaunchConfiguration('namespace').perform(context)
#     node_name = LaunchConfiguration('node_name').perform(context)
#     debug     = LaunchConfiguration('debug').perform(context)
#     custom_params_path = LaunchConfiguration('params_file').perform(context)

#     # Load default and override params
#     params = load_yaml(_DEFAULT_PARAMS_FILE)
#     if os.path.isfile(custom_params_path):
#         params.update(load_yaml(custom_params_path))

#     # Add launch-time override
#     params['debug'] = debug == 'true'

#     node = Node(
#         package='microstrain_inertial_driver',
#         executable='microstrain_inertial_driver_node',
#         name=node_name,
#         namespace=namespace,
#         parameters=[params]
#     )

#     return [node]

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument('namespace', default_value='/', description='Namespace'),
#         DeclareLaunchArgument('node_name', default_value=_PACKAGE_NAME, description='Node name'),
#         DeclareLaunchArgument('debug', default_value='false', description='Debug mode'),
#         DeclareLaunchArgument('params_file', default_value=_DEFAULT_PARAMS_FILE, description='Override parameter file'),
#         SetEnvironmentVariable('MICROSTRAIN_INERTIAL_DEBUG', LaunchConfiguration('debug')),
#         OpaqueFunction(function=launch_setup),
#     ])
