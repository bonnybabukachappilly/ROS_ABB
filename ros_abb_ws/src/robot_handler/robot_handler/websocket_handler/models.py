from typing import Any, NamedTuple

from rclpy.node import Publisher

from robot_handler.api_handler.models import APIClientModel


class WSConnection(NamedTuple):
    """Websocket connection model"""
    url: str
    header: dict[str, Any]
    protocol: list[str]
    publisher: Publisher
    api_client: APIClientModel
