import json
from collections.abc import Callable
from contextlib import suppress

import rclpy
from communications.srv import APIRequests  # type: ignore
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.service import Service
from requests import Response, Session
from requests.auth import HTTPBasicAuth, HTTPDigestAuth

from api_client.api.api_handler import APIHandler
from api_client.api.models import APIConnection


class ApiClient(Node):
    """
    Represents a client for interacting with an API.

    This class provides methods for making API requests and handling
        the responses.

    Args:
        Node: The base class for creating a ROS node.

    Attributes:
        api_handler (APIHandler):
            An instance of the APIHandler class for handling API requests.
        api_service (Service):
            A ROS service for receiving API requests.

    """

    def __init__(self) -> None:
        super().__init__('api_client', namespace='api_client')
        self.get_logger().info('API Client node has been started')

        self.declare_parameter("robot_host", "http://localhost:9900")
        self.declare_parameter("username", "default_user")
        self.declare_parameter("password", "default_pass")
        self.declare_parameter("auth_type", "basic")

        _robot_host: str = str(self.get_parameter("robot_host").value)
        _username: str = str(self.get_parameter("username").value)
        _password: str = str(self.get_parameter("password").value)
        _auth_type: str = str(self.get_parameter("auth_type").value)

        api_session = Session()

        api_conn = APIConnection(
            host=_robot_host,
            session=api_session,
            auth=HTTPBasicAuth(
                username=_username, password=_password
            ) if _auth_type == 'basic' else HTTPDigestAuth(
                username=_username, password=_password)
        )

        self.api_handler: APIHandler = APIHandler(api_conn)

        self.api_service: Service = self.create_service(
            srv_type=APIRequests,
            srv_name='api_requests',
            callback=self.api_request_callback
        )

    def __get_api_type(self, api_type: str) -> Callable[..., Response] | None:
        """
        Get the corresponding API request method based on the given API type.

        Args:
            api_type (str): The type of the API request.

        Returns:
            Callable[..., Response] | None: The corresponding API request
                method, or None if the API type is invalid.

        """
        match api_type:
            case "GET":
                return self.api_handler.get
            case "POST":
                return self.api_handler.post
            case "DELETE":
                return self.api_handler.delete
            case _:
                return None

    def __api_request_failed(self, response, msg: str) -> APIRequests.Response:
        """
        Handle a failed API request.

        Args:
            response: The response object to be modified.
            msg (str): The error message.

        Returns:
            APIRequests.Response: The modified response object.

        """
        response.success = False
        response.response = msg
        return response

    def api_request_callback(self, request, response) -> APIRequests.Response:
        """
        Callback function for handling API requests.

        Args:
            request: The request object containing the API request details.
            response: The response object to be modified.

        Returns:
            APIRequests.Response: The modified response object.

        """
        api_type: str = request.type
        data: str = request.data
        url: str = request.url
        headers: str = request.header
        code: str = request.code

        self.get_logger().info('Request received')

        api_request: Callable[..., Response] | None = self.__get_api_type(
            api_type=api_type
        )

        if api_request is None:
            return self.__api_request_failed(
                response, 'Invalid API type'
            )
        api_response: Response = api_request(
            url, json.loads(data), json.loads(headers))

        self.get_logger().info(f'Response status : {api_response.status_code}')

        if api_response.status_code != code:
            return self.__api_request_failed(
                response, response.response
            )

        cookies = {
            cookie.name: cookie.value for cookie in api_response.cookies
        }

        response.success = True
        response.cookies = json.dumps(cookies)
        response.header = json.dumps(dict(api_response.headers))
        response.response = api_response.content.decode('utf-8')

        return response


def main(args=None) -> None:
    """
    Entry point of the API service.

    Args:
        args (List[str], optional): Command-line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    api_client = ApiClient()

    try:
        rclpy.spin(api_client)
    except KeyboardInterrupt:
        print('API Client node has been stopped')
    finally:
        api_client.destroy_node()
        with suppress(RCLError):
            rclpy.shutdown()
