import json
import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the API service.

    This function creates and configures the launch description for the
    `api_service_handler` node, including loading parameters from a JSON file.

    Returns:
        LaunchDescription: The launch description object.
    """

    launch_dec = LaunchDescription()

    # Path to the config file
    config_file: str = os.path.join(
        get_package_share_directory('robot_handler'),
        'config',
        'robot_params.json'
    )

    with open(config_file) as config:
        configuration: dict[str, Any] = json.load(config)

    node_config: dict[str, Any] = {
        'urls.base_url': configuration['urls']['base_url'],
        'urls.system_info': configuration['urls']['system_info'],
        'urls.subscribe': configuration['urls']['subscribe'],
        'urls.joint_target': configuration['urls']['joint_target'],
        'websocket.subscriptions': configuration['urls']['subscriptions'],
    }

    api_service_handler = Node(
        package='robot_handler',
        executable='api_service_handler',
        name='api_service_handler',
        parameters=[node_config],
        on_exit=Shutdown(),
    )

    # Add the node to the launch description
    launch_dec.add_action(api_service_handler)

    return launch_dec
