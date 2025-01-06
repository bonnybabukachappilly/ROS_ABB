from io import TextIOWrapper
import os
from xml.dom.minidom import Document
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path: str = os.path.join(
        get_package_share_directory('abb_gofa_description')
    )
    xacro_file: str = os.path.join(pkg_path, 'urdf', 'abb_gofa.urdf.xacro')
    robot_description_config: Document | TextIOWrapper = xacro.process_file(
        xacro_file)

    # Create a robot_state_publisher node
    params = {
        'robot_description': robot_description_config.toxml(),  # type: ignore
        'use_sim_time': use_sim_time,
    }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory(
                'abb_gofa_description'), 'rviz', 'abb_gofa.rviz')]
    )

    # Launch!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use sim time if true',
            ),
            node_robot_state_publisher,
            # joint_state_publisher_gui,
            rviz2
        ]
    )
