import json
import os
from contextlib import suppress

import rclpy
from communications.srv import APISrvReq  # type: ignore
from dotenv import load_dotenv
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Any, Node
from rclpy.service import Service
from requests.auth import HTTPBasicAuth
from requests.models import Response

from robot_handler.api_handler.handler import APIClient
from robot_handler.api_handler.models import APIConnection, APIRequest
from robot_handler.common.logger import create_logger, get_logger


class APIServiceHandler(Node):
    """
    Class representing the API service handler.

    This class handles the API service requests and provides the necessary
    functionality to process the requests and generate the corresponding
    responses.

    Args:
        Node: The base class for creating a ROS node.

    Attributes:
        api (APIClient):
            The API client used for making API requests.
        api_service (Service):
            The ROS service for handling API requests.
    """

    def __init__(self) -> None:
        """
        Initialize the API service handler.

        This method sets up the API connection and initializes the API client.
        It also creates a service for handling API requests.
        """
        super().__init__('api_service_handler')

        api_username: str = os.getenv('ROBOT_USERNAME', '')
        api_password: str = os.getenv('ROBOT_PASSWORD', '')

        # Parameters
        self.declare_parameter('urls.base_url', '')

        base_url: str = self.get_parameter(
            'urls.base_url').get_parameter_value().string_value

        api_connection = APIConnection(
            host=f'http://{base_url}',
            auth=HTTPBasicAuth(username=api_username, password=api_password)
        )

        self.api = APIClient(api=api_connection)

        self.api.header = {
            "Accept": "application/hal+json;v=2.0",
            "Connection": "keep-alive",
        }

        self.api_service: Service = self.create_service(
            srv_type=APISrvReq,
            srv_name='api_service',
            callback=self.api_service_callback
        )

        get_logger().info(f'base_url: {base_url}')
        get_logger().info(f'api_username: {api_username}')

    def __get_url(self, key: str) -> str:
        """
        Get the URL based on the provided key.

        Args:
            key (str):
                The key to determine the URL.

        Returns:
            str:
                The URL corresponding to the provided key.

        Raises:
            ValueError:
                If the key is invalid.
        """
        match key:
            case 'speedratio':
                return "/rw/panel/speedratio"
            case 'ctrlstate':
                return "/rw/panel/ctrl-state"
            case 'opmode':
                return "/rw/panel/opmode"
            case 'ctrlexecstate':
                return "/rw/rapid/execution"
            case 'cycle':
                return "/rw/rapid/execution"
            case _:
                raise ValueError('Invalid key')

    def api_service_callback(self, request, response) -> APISrvReq.Response:
        """
        Callback function for the API service.

        Args:
            request:
                The request object containing the key.
            response:
                The response object to be filled.

        Returns:
            APISrvReq.Response:
                The response object with the result of the API request.

        Raises:
            ValueError:
                If the key is invalid.
        """

        try:
            _url: str = self.__get_url(request.key)
        except ValueError:
            return self.srv_response(
                response, False, 'Invalid key'
            )

        _request = APIRequest(
            api_type='GET',
            api_data=None,
            api_url=_url,
            api_headers=None,
            expected_code=200
        )

        _response: Response | None = self.api.process_api_request(
            request=_request)

        if _response is None:
            return self.srv_response(
                response, False, 'Invalid Response Code'
            )

        decoded: Any = json.loads(_response.content.decode())
        data: dict[str, str] | None = decoded.get('state')[0]

        if data is None:
            return self.srv_response(
                response, False, 'Invalid Response Data'
            )

        try:
            return self.srv_response(
                response, True, f'{request.key}:{data[request._key]}'
            )
        except KeyError:
            get_logger().info(f"Data: {data}")
            return self.srv_response(
                response, False, 'Corrupted Data received'
            )

    def srv_response(
            self, response: APISrvReq.Response,
            status: bool, msg: str) -> APISrvReq.Response:
        """
        Sets the status and message of the response object and returns it.

        Args:
            response (APISrvReq.Response):
                The response object to modify.
            status (bool):
                The status value to set.
            msg (str):
                The message to set.

        Returns:
            APISrvReq.Response:
                The modified response object.
        """
        response.status = status
        response.message = msg
        return response


def main(args=None) -> None:
    """
    Entry point of the program.

    Args:
        args (List[str], optional): Command-line arguments. Defaults to None.
    """
    load_dotenv()
    create_logger()

    rclpy.init(args=args)
    api_srv_handler = APIServiceHandler()

    try:
        get_logger().info('Starting API Service Handler Node')
        rclpy.spin(api_srv_handler)

    finally:
        get_logger().info('Shutting down: api_srv_handler')
        api_srv_handler.destroy_node()

        get_logger().info('Shutting down: rclpy')
        with suppress(RCLError):
            rclpy.shutdown()
