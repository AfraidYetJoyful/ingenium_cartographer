import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  
  launch_description = []
  # print(yaml.load(open('cartographer_config/microstrain_config.yaml', 'r'), Loader=yaml.FullLoader))
  microstrain_node = Node(
    package    = 'microstrain_inertial_driver',
    executable = "microstrain_inertial_driver_node",
    name       = 'microstrain_inertial_driver_node',
    namespace  = '',
    parameters = list(yaml.load(open('cartographer_config/microstrain_config.yaml', 'r'), Loader=yaml.FullLoader))
  )

  launch_description.append(microstrain_node)
  return LaunchDescription(launch_description)
  

if __name__ == "__main__":
  print(generate_launch_description())
 
