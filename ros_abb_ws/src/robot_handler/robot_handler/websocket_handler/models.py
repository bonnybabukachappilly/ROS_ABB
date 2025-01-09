from typing import Any, NamedTuple

from rclpy.node import Publisher

from robot_handler.api_handler.models import APIClientModel


class WSConnection(NamedTuple):
    """
    Represents a WebSocket connection.

    Attributes:
        url (str):
            The URL of the WebSocket connection.
        header (dict[str, Any]):
            The header information for the WebSocket connection.
        protocol (list[str]):
            The list of protocols supported by the WebSocket connection.
        publisher (Publisher):
            The publisher object associated with the WebSocket connection.
        api_client (APIClientModel):
            The API client model associated with the WebSocket connection.
    """
    url: str
    header: dict[str, Any]
    protocol: list[str]
    publisher: Publisher
    api_client: APIClientModel
