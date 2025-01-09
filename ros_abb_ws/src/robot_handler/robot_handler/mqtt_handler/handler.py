from time import sleep

from paho.mqtt.client import Client, MQTTMessage
from paho.mqtt.reasoncodes import ReasonCode  # type: ignore

from robot_handler.common.logger import get_logger
from robot_handler.mqtt_handler.models import MQTTAPIModel, MQTTClientModel
from robot_handler.mqtt_handler.utils import RPCHandler, publish_robot_info


class MQTTService:
    """
    A class representing an MQTT service.

    This class provides methods to connect to an MQTT broker, handle
    callbacks for various MQTT events, and interact with the MQTT client.

    Attributes:
        FIRST_RECONNECT_DELAY (int):
            The delay in seconds before the first
            reconnection attempt.
        RECONNECT_RATE (int):
            The rate at which the reconnection delay
            increases.
        MAX_RECONNECT_COUNT (int):
            The maximum number of reconnection attempts.
        MAX_RECONNECT_DELAY (int):
            The maximum delay in seconds between
            reconnection attempts.
        TERMINATE_CONNECTION (bool):
            A flag indicating whether the connection
            should be terminated.
        __slots__ (list[str]):
            A list of attribute names that the class can
            have. This reduces memory usage and improves performance.

    Properties:
        client (Client):
            The MQTT client instance.
        model (MQTTClientModel):
            The MQTT client model.
        api (MQTTAPIModel):
            The MQTT API model.

    Methods:
        __init__():
            Initializes the MQTTService instance.
        __on_connect():
            Callback function called when the MQTT client
            successfully connects to the broker.
        __on_publish():
            Callback function triggered when a message is
            successfully published.
        __on_message():
            Callback function called when a message is received.
        __on_disconnect():
            Callback function called when the MQTT client gets
            disconnected.
        connect():
            Connects to the MQTT broker using the provided configuration.
        spin():
            Starts the MQTT client and enters a loop to handle incoming
            messages indefinitely.
        shutdown():
            Stops the MQTT client and disconnects from the broker.
    """

    FIRST_RECONNECT_DELAY: int = 1
    RECONNECT_RATE: int = 2
    MAX_RECONNECT_COUNT: int = 12
    MAX_RECONNECT_DELAY: int = 60
    TERMINATE_CONNECTION: bool = False

    __slots__: list[str] = [
        '__first_connection',
        "__client",
        "__model",
        "__api",
        "__rpc",
    ]

    def __init__(self) -> None:
        """
        Initializes the MQTTService instance.
        """
        self.__client: Client
        self.__model: MQTTClientModel
        self.__api: MQTTAPIModel
        self.__rpc: RPCHandler

        self.__first_connection = True

    @property
    def client(self) -> Client:
        """
        Get the MQTT client instance.

        Returns:
            The MQTT client instance.
        """
        return self.__client

    @property
    def model(self) -> MQTTClientModel:
        """
        Get the MQTT client model.

        Returns:
            The MQTT client model.
        """
        return self.__model

    @model.setter
    def model(self, model: MQTTClientModel) -> None:
        """
        Set the MQTT client model.

        Args:
            model (MQTTClientModel): The MQTT client model.
        """
        self.__model = model

    @property
    def api(self) -> MQTTAPIModel:
        """
        Get the MQTT API model.

        Returns:
            The MQTT API model.
        """
        return self.__api

    @api.setter
    def api(self, api: MQTTAPIModel) -> None:
        """
        Set the MQTT API model.

        Args:
            api (MQTTAPIModel): The MQTT API model.
        """
        self.__api = api

    def __on_connect(self, *args) -> None:
        """
        Callback function called when the MQTT client successfully connects to
        the broker.

        Args:
            *args: Variable-length argument list containing the following:
                - client (Client): The MQTT client instance.
                - rc (ReasonCode): The reason code indicating the result of
                    the publish operation.
        """
        client: Client = args[0]
        rc: ReasonCode = args[3]

        client_name: str = client._client_id.decode()  # type: ignore

        get_logger().info(f"{client_name} connected with rc: {rc}")

        if self.__first_connection:
            self.__first_connection = False
            publish_robot_info(
                api=self.__api.api,
                api_url=self.__api.robot_info_url,
                mqtt_url=self.__api.attribute_url,
                qos=self.__api.qos,
                client=self.__client
            )

    def __on_publish(self, *args) -> None:
        """
        Callback function triggered when a message is successfully published.

        Args:
            *args: Variable-length argument list containing the following:
                - client (Client): The MQTT client instance.
                - rc (ReasonCode): The reason code indicating the result of
                    the publish operation.
        """
        client: Client = args[0]
        rc: ReasonCode = args[3]

        client_name: str = client._client_id.decode()  # type: ignore

        get_logger().info(f"{client_name} published with rc: {rc}")

    def __on_message(self, *args) -> None:
        """
        Callback function called when a message is received.

        Args:
            *args: Variable-length argument list containing the following:
                - client (Client): The MQTT client instance.
                - message (MQTTMessage): The received message.
        """
        client: Client = args[0]
        message: MQTTMessage = args[2]

        client_name: str = client._client_id.decode()  # type: ignore
        data = message.payload.decode()

        get_logger().info(f'{client_name} > {data}')

        self.__rpc.handle_request(request=data)

    def __on_disconnect(self, *args) -> None:
        """
        Callback function called when the MQTT client gets disconnected.

        Args:
            *args: Variable-length argument list containing the following:
                - client (Client): The MQTT client instance.
                - rc (ReasonCode): The reason code indicating the result of
                    the publish operation.
        """
        cls: type[MQTTService] = self.__class__

        client: Client = args[0]
        rc: ReasonCode = args[3]

        client_name: str = client._client_id.decode()  # type: ignore

        get_logger().warning(f"{client_name} disconnected with rc: {rc}")

        rconn_count: int = 0
        rconn_delay: int = cls.FIRST_RECONNECT_DELAY

        if cls.TERMINATE_CONNECTION:
            return

        while rconn_count < cls.MAX_RECONNECT_COUNT:
            get_logger().warning(f"Trying to reconnect to {client_name}")
            sleep(rconn_delay)

            try:
                client.reconnect()
                get_logger().info(f"{client_name} reconnected successfully")
                return
            except Exception as e:
                get_logger().error(f"Reconnect failed: {e}")

            rconn_count += 1
            rconn_delay = min(
                rconn_delay * cls.RECONNECT_RATE,
                cls.MAX_RECONNECT_DELAY
            )

        get_logger().error(f"Failed to reconnect to {client_name}")

    def connect(self) -> None:
        """
        Connects to the MQTT broker using the provided configuration.

        This method initializes the MQTT client, sets up the necessary
        callbacks, and establishes a connection to the broker.

        Raises:
            Any exceptions raised by the MQTT client library during connection.
        """
        self.__client = Client(
            callback_api_version=self.__model.version,  # type: ignore
            client_id=self.__model.client_id,
        )

        self.__model.auth.authenticate(self.__client)

        self.__client.on_connect = self.__on_connect
        self.__client.on_publish = self.__on_publish
        self.__client.on_message = self.__on_message
        self.__client.on_disconnect = self.__on_disconnect

        self.__client.connect(
            host=self.__model.broker,
            port=self.__model.port,
            keepalive=self.__model.timeout
        )

        self.__client.subscribe(
            topic=self.__model.sub_request
        )

        self.__rpc = RPCHandler(
            client=self.__client,
            topic=self.__model.sub_response,
            qos=self.__api.qos
        )

    def spin(self) -> None:
        """
        Start the MQTT client and enter a loop to handle incoming messages
        indefinitely.
        """
        self.__client.loop_forever()

    def shutdown(self) -> None:
        """
        Stops the MQTT client and disconnects from the broker.

        This method stops the MQTT client's event loop and disconnects from
        the MQTT broker.

        Returns:
            None
        """
        self.__client.loop_stop()
        self.__client.disconnect()
