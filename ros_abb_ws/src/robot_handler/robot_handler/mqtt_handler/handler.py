from time import sleep

from paho.mqtt.client import Client
from paho.mqtt.reasoncodes import ReasonCode  # type: ignore

from robot_handler.common.logger import get_logger
from robot_handler.mqtt_handler.models import MQTTAPIModel, MQTTClientModel
from robot_handler.mqtt_handler.utils import get_robot_info


class MQTTService:
    FIRST_RECONNECT_DELAY: int = 1
    RECONNECT_RATE: int = 2
    MAX_RECONNECT_COUNT: int = 12
    MAX_RECONNECT_DELAY: int = 60
    TERMINATE_CONNECTION: bool = False

    __slots__: list[str] = [
        "__client",
        "__model",
        "__api",
    ]

    def __init__(self) -> None:
        self.__client: Client
        self.__model: MQTTClientModel
        self.__api: MQTTAPIModel

    @property
    def client(self) -> Client:
        return self.__client

    @property
    def model(self) -> MQTTClientModel:
        return self.__model

    @model.setter
    def model(self, model: MQTTClientModel) -> None:
        self.__model = model

    @property
    def api(self) -> MQTTAPIModel:
        return self.__api

    @api.setter
    def api(self, api: MQTTAPIModel) -> None:
        self.__api = api

    def __on_connect(self, *args) -> None:
        client: Client = args[0]
        rc: ReasonCode = args[3]

        client_name: str = client._client_id.decode()  # type: ignore

        get_logger().info(f"{client_name} connected with rc: {rc}")

        get_robot_info(
            api=self.__api.api,
            api_url=self.__api.robot_info_url,
            mqtt_url=self.__api.attribute_url,
            qos=self.__api.qos,
            client=self.__client
        )

    def __on_publish(self, *args) -> None:
        client: Client = args[0]
        rc: ReasonCode = args[3]

        client_name: str = client._client_id.decode()  # type: ignore

        get_logger().info(f"{client_name} published with rc: {rc}")

    def __on_message(self, *args) -> None:
        ...

    def __on_disconnect(self, *args) -> None:
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

    def spin(self) -> None:
        self.__client.loop_forever()

    def shutdown(self) -> None:
        self.__client.loop_stop()
        self.__client.disconnect()
