import json
from typing import Any

from paho.mqtt.client import Client
from requests.models import Response

from robot_handler.api_handler.models import APIClientModel, APIRequest
from robot_handler.api_handler.utils import format_api_model
from robot_handler.common.logger import get_logger
from robot_handler.common.mappings import ROBOT_INFO_MAPPING


def publish_robot_info(
        api: APIClientModel,
        api_url: str, mqtt_url: str,
        qos: int,  client: Client) -> None:
    """
    Retrieves robot information from the API and publishes it to the
    MQTT broker.

    Args:
        api (APIClientModel):
            The API client used to make the request.
        api_url (str):
            The URL of the API endpoint to retrieve the robot information.
        mqtt_url (str):
            The MQTT topic to publish the robot information to.
        qos (int):
            The quality of service level for the MQTT message.
        client (Client):
            The MQTT client used to publish the message.

    Returns:
        None
    """
    request = APIRequest(
        api_type='GET',
        api_data=None,
        api_url=api_url,
        api_headers=None,
        expected_code=200
    )

    response: Response | None = api.process_api_request(request)

    data: dict[str, Any] | None = format_api_model(
        response=response,
        model=ROBOT_INFO_MAPPING
    )

    client.publish(
        topic=mqtt_url,
        payload=json.dumps(data),
        qos=1
    )


class RPCHandler:
    __slots__: list[str] = [
        "__client",
        "__topic",
        "__qos"
    ]

    def __init__(self, client: Client, topic: str, qos: int) -> None:
        self.__client: Client = client
        self.__topic: str = topic
        self.__qos: int = qos

    def __get_handler(self, request: str):
        match request:
            case 'checkExecutionState':
                return
            case _:
                raise ValueError('Invalid key')

    def handle_request(self, request: str) -> None:
        data = json.loads(request)
        try:
            self.__get_handler(request=data['method'])
        except ValueError:
            get_logger().info(f"Invalid RPC request: {data['method']}")

        self.__client.publish(
            topic=self.__topic,
            payload=json.dumps({"checkStatus": True}),
            qos=self.__qos
        )
