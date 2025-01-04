from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
from websocket import WebSocketApp

from websocket_client.ws_client.models import WSConnection


class WSClient(Node):
    def __init__(self) -> None:
        super().__init__('websocket_client', namespace='ws_client')
        self.__ws_connection: WSConnection

        self.publisher: Publisher = self.create_publisher(
            msg_type=String,
            topic='rws_subscription',
            qos_profile=10
        )

    @property
    def ws_connection(self) -> WSConnection:
        return self.__ws_connection

    @ws_connection.setter
    def ws_connection(self, ws: WSConnection) -> None:
        self.__ws_connection = ws

    def __on_open(self, *args) -> None: ...

    def __on_message(self, *args) -> None: ...

    def __on_close(self, *_) -> None: ...

    def __on_error(self, *_) -> None: ...

    def create_connect(self, conn: WSConnection) -> None:

        self.__client = WebSocketApp(
            url=self.__ws_connection.url,
            header=self.__ws_connection.header,
            subprotocols=self.__ws_connection.protocol,
            on_open=self.__on_open,
            on_message=self.__on_message,
            on_close=self.__on_close,
            on_error=self.__on_error,
        )

        self.__subscription_poll = conn.url.split("/")[-1]
