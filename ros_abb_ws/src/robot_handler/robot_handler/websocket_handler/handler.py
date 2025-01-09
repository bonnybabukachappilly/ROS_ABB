from requests.models import Response
from std_msgs.msg import String
from websocket import WebSocketApp
from websocket._app import WebSocketApp as TWebSocketApp

from robot_handler.api_handler.models import APIClientModel, APIRequest
from robot_handler.common.logger import get_logger
from robot_handler.websocket_handler.models import WSConnection
from robot_handler.xml_handler.handler import XMLHandler


class WSClient:
    """
    Represents a WebSocket client for handling WebSocket connections.

    Attributes:
        ws_conn (WSConnection):
            The WebSocket connection object.
        xml_handler (XMLHandler):
            The XML handler object for parsing XML messages.
    """

    __slots__: list[str] = [
        "__client",
        "__poll_id",
        "ws_conn",
        "xml_handler"
    ]

    def __init__(self, ws_connection: WSConnection) -> None:
        """
        Initializes a new instance of the WSClient class.

        Args:
            ws_connection (WSConnection):
                The WebSocket connection object.
        """
        self.ws_conn: WSConnection = ws_connection
        self.xml_handler: XMLHandler = XMLHandler(
            api=self.ws_conn.api_client
        )
        self.__client: TWebSocketApp

    def __on_open(self, *args) -> None:
        """
        Event handler for WebSocket connection open event.
        """
        ws: TWebSocketApp = args[0]
        get_logger().info(f"Websocket connection opened on '{ws.url}'")

    def __on_message(self, *args) -> None:
        """
        Event handler for WebSocket message event.
        """
        msg: str = self.xml_handler.parse(args[1])
        get_logger().debug(msg)

        data = String()
        data.data = msg
        self.ws_conn.publisher.publish(data)

    def __on_close(self, *_) -> None:
        """
        Event handler for WebSocket connection close event.
        """
        get_logger().info("Websocket connection closed")
        _request = APIRequest(
            api_type="DELETE",
            api_data=None,
            api_url=f"/subscription/{self.__poll_id}",
            api_headers=None,
            expected_code=200
        )

        api_request: APIClientModel = self.ws_conn.api_client

        _response: Response | None = api_request.process_api_request(
            _request)

        if _response is not None:
            get_logger().info(
                f"Subscription with id {self.__poll_id} "
                + "has been terminated"
            )
        else:
            get_logger().info(
                f"Subscription with id {self.__poll_id} "
                + "cannot be terminated"
            )

    def __on_error(self, *_) -> None:
        """
        Event handler for WebSocket error event.
        """
        ...

    def create_connect(self) -> None:
        """
        Creates a WebSocket connection.
        """
        get_logger().info(f"Trying to connect to '{self.ws_conn.url}'")

        self.__client = WebSocketApp(
            url=self.ws_conn.url,
            header=self.ws_conn.header,
            subprotocols=self.ws_conn.protocol,
            on_open=self.__on_open,
            on_message=self.__on_message,
            on_close=self.__on_close,
            on_error=self.__on_error,
        )

        self.__poll_id: str = self.ws_conn.url.split("/")[-1]

    def shutdown(self) -> None:
        """
        Shuts down the WebSocket connection.
        """
        self.__client.close()

    def spin(self) -> None:
        """
        Starts the WebSocket client and runs it indefinitely.
        """
        self.__client.run_forever(reconnect=5)
