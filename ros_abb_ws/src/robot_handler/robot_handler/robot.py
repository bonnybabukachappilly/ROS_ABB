from contextlib import suppress
from logging import Logger

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node

from robot_handler.common.logger import create_logger, get_logger


class RobotHandler(Node):
    def __init__(self) -> None:
        super().__init__('robot_handler')
        self.get_logger().info('Robot Handler Node Started')


def main(args=None) -> None:
    create_logger()
    log: Logger = get_logger()

    rclpy.init(args=args)
    robot_handler = RobotHandler()

    try:
        log.info('Robot Handler Node Started')
        rclpy.spin(robot_handler)

    finally:
        log.info('Robot Handler Node Stopped')
        robot_handler.destroy_node()

        with suppress(RCLError):
            log.info('Shutting down ROS2')
            rclpy.shutdown()
