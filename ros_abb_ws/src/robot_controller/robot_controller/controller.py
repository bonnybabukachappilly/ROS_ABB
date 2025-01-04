import rclpy
from rclpy.node import Node

from robot_controller.utils.models import APISubscriptionRequest
from robot_controller.utils.subscriptions import create_subscription


class Controller(Node):
    def __init__(self) -> None:
        super().__init__('controller')
        self.get_logger().info('Controller node has been started')

        # Create Subscription
        rws_subscription: APISubscriptionRequest = create_subscription()

        # Request subscription

        # Request Websocket


def main(args=None) -> None:
    rclpy.init(args=args)
    controller = Controller()

    try:
        rclpy.spin(controller)
    finally:
        controller.destroy_node()
        rclpy.shutdown()
