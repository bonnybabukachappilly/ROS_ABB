import os
from contextlib import suppress
from threading import Thread

import rclpy
from dotenv import load_dotenv
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.publisher import Publisher
from requests.auth import HTTPBasicAuth
from requests.models import Response
from std_msgs.msg import String

from robot_handler.api_handler.handler import APIClient
from robot_handler.api_handler.models import APIConnection, APIRequest
from robot_handler.common.logger import create_logger, get_logger
from robot_handler.websocket_handler.handler import WSClient
from robot_handler.websocket_handler.models import WSConnection
from robot_handler.websocket_handler.utils import create_subscription


class WSHandler(Node):
    """
    This class represents a WebSocket handler node in the ROS system.
    It handles the creation of API connections and WebSocket connections,
    as well as managing subscriptions and publishing robot logs.

    Attributes:
        __api (APIClient):
            An instance of the APIClient class for making API requests.
        __ws_connection (WSConnection):
            An instance of the WSConnection class for WebSocket communication.
        publisher (Publisher):
            A publisher object for publishing robot logs.

    """

    def __init__(self) -> None:
        """
        Initializes the WebsocketHandler class.

        This method is called when an instance of the WebsocketHandler class
        is created. It sets up the necessary parameters, creates an API client,
        and establishes a websocket connection.
        """
        super().__init__('websocket_handler')
        get_logger().info('Websocket Handler Node Started')

        # Parameters
        self.declare_parameter('urls.base_url', '')
        self.declare_parameter('websocket.url', '')
        self.declare_parameter('websocket.subscriptions', ['', ''])
        self.declare_parameter('urls.subscribe', '')

        self.__api: APIClient
        self.__ws_connection: WSConnection

        self.publisher: Publisher = self.create_publisher(
            msg_type=String,
            topic='robot_logs',
            qos_profile=10
        )

        self.create_api()

        self.create_websocket(
            response=self.create_rws_subscription()
        )

    @property
    def ws_connection(self) -> WSConnection:
        return self.__ws_connection

    def create_api(self) -> None:
        """
        Creates an API connection using the provided base URL and
        authentication credentials. Sets the created API connection
        as the instance variable __api.

        """

        api_username: str = os.getenv('ROBOT_USERNAME', '')
        api_password: str = os.getenv('ROBOT_PASSWORD', '')

        base_url: str = self.get_parameter(
            'urls.base_url').get_parameter_value().string_value

        api_connection = APIConnection(
            host=f'http://{base_url}',
            auth=HTTPBasicAuth(username=api_username, password=api_password)
        )

        self.__api = APIClient(api=api_connection)

        self.__api.header = {
            "Accept": "application/hal+json;v=2.0",
            "Connection": "keep-alive",
        }

    def create_rws_subscription(self) -> Response:
        """
        Creates a subscription request for the WebSocket connection.
        Retrieves the WebSocket subscriptions and subscription URL
        from parameters. Sends the subscription request to the API
        and returns the response.

        Returns:
            Response:
                The response received from the API after sending the
                subscription request.

        Raises:
            ValueError:
                If the response is None, indicating an invalid response code.

        """

        ws_subscriptions: list[str] = self.get_parameter(  # type: ignore
            'websocket.subscriptions').get_parameter_value().string_array_value

        subscription_url: str = self.get_parameter(
            'urls.subscribe').get_parameter_value().string_value

        sub_request: APIRequest = create_subscription(
            subscriptions=ws_subscriptions,
            url=subscription_url
        )

        _response: Response | None = self.__api.process_api_request(
            request=sub_request)
        if _response is None:
            raise ValueError('Invalid Response Code')

        return _response

    def create_websocket(self, response: Response) -> None:
        """
        Creates a WebSocket connection using the provided response
        from the API. Sets the created WebSocket connection as the
        instance variable __ws_connection.

        Args:
            response (Response):
                The response received from the API after sending
                the subscription request.

        """

        _session: str = f"-http-session-={response.cookies['-http-session-']}"
        _abbcx: str = f"ABBCX={response.cookies['ABBCX']}"

        self.__ws_connection = WSConnection(
            url=response.headers["Location"],
            header={"Cookie": f"{_session}; {_abbcx}"},
            protocol=["rws_subscription"],
            publisher=self.publisher,
            api_client=self.__api
        )


def main(args=None) -> None:
    """
    Entry point of the program.

    Args:
        args (List[str], optional):
            Command-line arguments. Defaults to None.
    """
    load_dotenv()

    create_logger()

    rclpy.init(args=args)
    ws_handler = WSHandler()

    ws_connection: WSConnection = ws_handler.ws_connection

    ws_client = WSClient(ws_connection=ws_connection)
    ws_client.create_connect()

    thread = Thread(target=ws_client.spin, daemon=True)

    try:
        get_logger().info('Starting Websocket Handler Node')
        thread.start()

        rclpy.spin(ws_handler)

    finally:
        terminate(ws_client, thread, ws_handler)


def terminate(ws_client, thread, ws_handler) -> None:
    """
    Terminate the WebSocket client, thread, and handler.

    Args:
        ws_client:
            The WebSocket client object.
        thread:
            The thread object.
        ws_handler:
            The WebSocket handler object.
    """
    get_logger().info('Shutting down: ws_client')
    ws_client.shutdown()

    get_logger().info('Waiting for thread termination')
    thread.join()

    get_logger().info('Shutting down: ws_handler')
    ws_handler.destroy_node()

    get_logger().info('Shutting down: rclpy')
    with suppress(RCLError):
        rclpy.shutdown()
