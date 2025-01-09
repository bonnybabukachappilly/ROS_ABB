from dataclasses import dataclass
from typing import NamedTuple, Protocol

from paho.mqtt.client import Client
from paho.mqtt.enums import CallbackAPIVersion  # type: ignore

from robot_handler.api_handler.models import APIClientModel


@dataclass
class Authentication(Protocol):
    """
    Represents an authentication protocol for a client.

    This class defines the interface for authenticating a client.

    Attributes:
        None

    Methods:
        authenticate: Authenticates the client.

    """

    def authenticate(self, client: Client) -> None:
        ...


@dataclass
class UserPassAuth:
    """
    Represents a user authentication object for MQTT client.

    Attributes:
        username (str): The username for authentication.
        password (str): The password for authentication.
    """

    username: str
    password: str

    def authenticate(self, client: Client) -> None:
        """
        Authenticates the MQTT client using the provided username and password.

        Args:
            client (paho.mqtt.client.Client): The MQTT client to authenticate.
        """
        client.username_pw_set(self.username, self.password)


@dataclass
class TokenAuth:
    """
    TokenAuth class represents an authentication mechanism using a token.

    Attributes:
        token (str): The authentication token.

    Methods:
        authenticate(client: Client) -> None:
            Authenticates the client using the provided token.
    """
    token: str

    def authenticate(self, client: Client) -> None:
        """
        Authenticates the client using the provided token.

        Args:
            client (Client):
                The MQTT client to authenticate.

        Returns:
            None
        """
        client.username_pw_set(self.token)


class MQTTClientModel(NamedTuple):
    """
    Represents the model for an MQTT client.

    Attributes:
        broker (str):
            The MQTT broker address.
        port (int):
            The port number for the MQTT broker.
        client_id (str):
            The client ID for the MQTT client.
        timeout (int):
            The timeout value for the MQTT client.
        auth (Authentication):
            The authentication details for the MQTT client.
        version (CallbackAPIVersion):
            The callback API version for the MQTT client.
    """
    broker: str
    port: int
    client_id: str
    timeout: int
    auth: Authentication
    version: CallbackAPIVersion


class MQTTAPIModel(NamedTuple):
    """
    Represents the MQTT API model.

    Attributes:
        api (APIClientModel):
            The API client model.
        robot_info_url (str):
            The URL for robot information.
        attribute_url (str):
            The URL for attributes.
        qos (int):
            The quality of service level.
    """
    api: APIClientModel
    robot_info_url: str
    attribute_url: str
    qos: int
