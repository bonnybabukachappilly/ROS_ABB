from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    launch_dec = LaunchDescription()

    rws_node = Node(
        package='robot_controller',
        executable='controller'
    )

    launch_dec.add_action(rws_node)

    return launch_dec
