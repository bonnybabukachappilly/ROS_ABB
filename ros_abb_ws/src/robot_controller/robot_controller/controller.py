import json
from contextlib import suppress
from typing import Any

import rclpy
from communications.srv import (
    APIRequests,  # type: ignore
    WSRequest,  # type: ignore
)
from rclpy import Future
from rclpy._rclpy_pybind11 import RCLError
from rclpy.client import Client
from rclpy.node import Node

from robot_controller.utils.models import APISubscriptionRequest, WSConnection
from robot_controller.utils.subscriptions import create_subscription


class Controller(Node):
    def __init__(self) -> None:
        super().__init__('robot_controller')
        self.get_logger().info('Controller node has been started')

        # Request subscription
        self.api_client: Client = self.create_client(
            srv_type=APIRequests, srv_name='/api_client/api_requests'
        )

        # Start subscription
        self.ws_client: Client = self.create_client(
            srv_type=WSRequest, srv_name='/ws_client/ws_services'
        )

        while not self.api_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('API service not available, waiting again.')

        while not self.ws_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WS service not available, waiting again.')

        self.handle_subscription()

    def register_websocket(self, ws: WSConnection) -> Any:
        _request: WSRequest.Request = WSRequest.Request()

        _request.url = ws.url
        _request.header = json.dumps(ws.header)
        _request.protocol = json.dumps(ws.protocol)

        future: Future = self.ws_client.call_async(_request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def register_subscription(self) -> Any:
        # Create Subscription
        rws_subscription: APISubscriptionRequest = create_subscription()

        _request: APIRequests.Request = APIRequests.Request()

        _request.type = rws_subscription.api_type
        _request.data = json.dumps(rws_subscription.api_data)
        _request.url = rws_subscription.api_url
        _request.header = json.dumps(rws_subscription.api_headers)
        _request.code = rws_subscription.api_code

        future: Future = self.api_client.call_async(_request)

        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def handle_subscription(self) -> None:
        result = self.register_subscription()

        if not result.success:
            self.get_logger().info(f'{result.response = }')
            return

        cookies = json.loads(result.cookies)
        header = json.loads(result.header)

        _session: str = f"-http-session-={cookies['-http-session-']}"
        _abbcx: str = f"ABBCX={cookies['ABBCX']}"

        ws = WSConnection(
            url=header["Location"],
            header={"Cookie": f"{_session}; {_abbcx}"},
            protocol=["rws_subscription"],
        )

        result = self.register_websocket(ws)

        if not result.success:
            self.get_logger().info('Failed to create subscription')
            return

        self.get_logger().info('Subscription created successfully')


def main(args=None) -> None:
    rclpy.init(args=args)
    controller = Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('API Client node has been stopped')
    finally:
        controller.destroy_node()
        with suppress(RCLError):
            rclpy.shutdown()
