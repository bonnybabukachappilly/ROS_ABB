from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    launch_dec = LaunchDescription()

    rws_node = Node(
        package='websocket_client',
        executable='ws_client'
    )

    launch_dec.add_action(rws_node)

    return launch_dec
