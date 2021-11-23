import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
 
  pub_node = Node(
    package='mros2_echoback', executable='pub_node', output='screen'
    )
  simple_udp_node = Node(
    package='simple_udp', executable='sub_udp', output='screen'
    )

  ld = LaunchDescription()
  ld.add_action(pub_node)
  ld.add_action(simple_udp_node)

  return ld