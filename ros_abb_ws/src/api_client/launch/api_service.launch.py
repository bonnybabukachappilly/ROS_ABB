import json
import os

from dotenv import load_dotenv
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    load_dotenv()

    rws_parameter: list[dict[str, str]] = [
        {
            "robot_host": "http://localhost:9900",
            "username": os.getenv("ROBOT_USERNAME", "robotics"),
            "password": os.getenv("ROBOT_PASSWORD", "robotics"),
            "header": json.dumps(
                {
                    "Accept": "application/hal+json;v=2.0",
                    "Connection": "keep-alive",
                }
            )
        }
    ]

    launch_dec = LaunchDescription()

    rws_node = Node(
        package='api_client',
        executable='api_client',
        parameters=rws_parameter,
    )

    launch_dec.add_action(rws_node)

    return launch_dec
