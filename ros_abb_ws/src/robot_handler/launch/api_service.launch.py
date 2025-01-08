import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    launch_dec = LaunchDescription()

    # Path to the YAML file
    config_file: str = os.path.join(
        get_package_share_directory('robot_handler'),
        'config',
        'robot_params.yaml'
    )

    # Define the node with the YAML parameters
    api_service_handler = Node(
        package='robot_handler',
        executable='api_service_handler',
        name='api_service_handler',
        parameters=[config_file],
        on_exit=Shutdown(),
    )

    # Add the node to the launch description
    launch_dec.add_action(api_service_handler)

    return launch_dec
