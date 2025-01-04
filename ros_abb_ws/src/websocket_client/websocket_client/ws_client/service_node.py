import json

from communications.srv import WSRequest  # type: ignore
from rclpy.node import Node
from rclpy.service import Service

from websocket_client.ws_client.models import WSConnection


class WSService(Node):
    def __init__(self) -> None:
        super().__init__('websocket_service', namespace='ws_client')

        _: Service = self.create_service(
            srv_type=WSRequest,
            srv_name='ws_services',
            callback=self.ws_client_callback
        )

        self.__ws_connection: WSConnection

    @property
    def ws_connection(self) -> WSConnection:
        return self.__ws_connection

    def ws_client_callback(self, request, response) -> WSRequest.Response:
        self.get_logger().info('Request received')

        self.__ws_connection = WSConnection(
            url=request.url,
            header=json.loads(request.header),
            protocol=json.loads(request.protocol)
        )

        response.success = True

        return response
