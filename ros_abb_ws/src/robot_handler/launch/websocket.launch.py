import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the websocket handler.

    This function creates and configures the launch description for the
    `websocket_handler` node, loading parameters from a YAML file and
    remapping the `robot_logs` topic to `mqtt_publish`.

    Returns:
        LaunchDescription: The launch description object.
    """

    launch_dec = LaunchDescription()

    # Path to the YAML file
    config_file: str = os.path.join(
        get_package_share_directory('robot_handler'),
        'config',
        'robot_params.yaml'
    )

    # Define the node with the YAML parameters
    websocket_handler = Node(
        package='robot_handler',
        executable='websocket_handler',
        name='websocket_handler',
        parameters=[config_file],
        on_exit=Shutdown(),
        remappings=[
            ("robot_logs", "mqtt_publish")
        ]
    )

    # Add the node to the launch description
    launch_dec.add_action(websocket_handler)

    return launch_dec
