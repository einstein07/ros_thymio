import os
import pathlib
import launch
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the config directory
    config_dir = os.path.join('/home/sindiso/ros2_ws/src/ros_thymio', 'config')
    param_config = os.path.join(config_dir, "config.yaml")
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["ros_thymio"]["ros__parameters"]
    
    return LaunchDescription([
        Node(
            package="ros_thymio",
            executable="thymio_node",
            name="controller",
            output="screen",
            parameters=[
                params
            ]
        )
    ])
