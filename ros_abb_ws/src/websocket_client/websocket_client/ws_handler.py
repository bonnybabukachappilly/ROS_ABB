from contextlib import suppress

import rclpy
from rclpy._rclpy_pybind11 import RCLError

from websocket_client.ws_client.client_node import WSClient
from websocket_client.ws_client.service_node import WSService


def main(args=None) -> None:
    rclpy.init(args=args)
    ws_service = WSService()
    ws_client = WSClient()

    try:
        rclpy.spin_once(ws_service)
        ws_client.ws_connection = ws_service.ws_connection
        ws_service.destroy_node()
        rclpy.spin(ws_client)

    except KeyboardInterrupt:
        print('API Client node has been stopped')

    finally:
        ws_client.destroy_node()
        with suppress(RCLError):
            rclpy.shutdown()
