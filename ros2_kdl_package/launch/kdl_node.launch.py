from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)


def generate_launch_description():

    
    config = os.path.join(get_package_share_directory('ros2_kdl_package'),
    "config",
    "params.yaml"
    )

    node = Node(
        package= 'ros2_kdl_package',
        name = 'ros2_kdl_node',
        executable = 'ros2_kdl_node_ex',
        parameters = [config]
    )

    nodes_to_start = [
        node
    ]
    
    return LaunchDescription(nodes_to_start) 