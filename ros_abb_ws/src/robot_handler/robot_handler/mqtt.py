import json
import os
from contextlib import suppress
from threading import Thread

import rclpy
from dotenv import load_dotenv
from paho.mqtt.enums import CallbackAPIVersion  # type: ignore
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.subscription import Subscription
from requests.auth import HTTPBasicAuth
from std_msgs.msg import String

from robot_handler.api_handler.handler import APIClient
from robot_handler.api_handler.models import APIConnection
from robot_handler.common.logger import create_logger, get_logger
from robot_handler.mqtt_handler.handler import MQTTService
from robot_handler.mqtt_handler.models import (
    Authentication,
    MQTTAPIModel,
    MQTTClientModel,
    TokenAuth,
    UserPassAuth,
)


class MQTTServiceHandler(Node):
    def __init__(self, mqtt: MQTTService) -> None:
        super().__init__('mqtt_service_handler')

        # Parameters
        self.declare_parameter('mqtt.broker', '')
        self.declare_parameter('mqtt.port', 0)
        self.declare_parameter('mqtt.timeout', 0)
        self.declare_parameter('mqtt.publish.telemetry', '')
        self.declare_parameter('mqtt.publish.attribute', '')
        self.declare_parameter('mqtt.publish.qos', 0)
        self.declare_parameter('urls.base_url', '')
        self.declare_parameter('urls.system_info', '')
        self.declare_parameter('api.header', "{}")

        _: Subscription = self.create_subscription(
            msg_type=String,
            topic='/mqtt_publish',
            callback=self.publish_msg,
            qos_profile=10
        )

        self.__api: MQTTAPIModel = self.__create_api()

        self.__mqtt: MQTTService = mqtt

        model: MQTTClientModel = self.__create_mqtt_model()
        self.__mqtt.model = model
        self.__mqtt.api = self.__api

    def publish_msg(self, msg: String) -> None:
        telemetry: str = self.get_parameter(
            'mqtt.publish.telemetry').get_parameter_value().string_value

        qos: int = self.get_parameter(
            'mqtt.publish.qos').get_parameter_value().integer_value

        get_logger().info(f'Publishing message: {msg.data}')
        self.__mqtt.client.publish(
            topic=telemetry,
            payload=msg.data,
            qos=qos,
        )

    def __create_api(self) -> MQTTAPIModel:

        api_username: str = os.getenv('ROBOT_USERNAME', '')
        api_password: str = os.getenv('ROBOT_PASSWORD', '')

        base_url: str = self.get_parameter(
            'urls.base_url').get_parameter_value().string_value

        system_info: str = self.get_parameter(
            'urls.system_info').get_parameter_value().string_value

        attribute_url: str = self.get_parameter(
            'mqtt.publish.attribute').get_parameter_value().string_value

        qos: int = self.get_parameter(
            'mqtt.publish.qos').get_parameter_value().integer_value

        header: dict[str, str] = json.loads(
            self.get_parameter(
                'api.header').get_parameter_value().string_value
        )

        api_connection = APIConnection(
            host=f'http://{base_url}',
            auth=HTTPBasicAuth(username=api_username, password=api_password)
        )

        api = APIClient(api=api_connection)

        api.header = header

        return MQTTAPIModel(
            api=api,
            robot_info_url=system_info,
            attribute_url=attribute_url,
            qos=qos
        )

    def __create_mqtt_model(self) -> MQTTClientModel:
        auth: Authentication

        mqtt_client_id: str = os.getenv('MQTT_CLIENT_ID', '')

        mqtt_username: str = os.getenv('MQTT_USERNAME', '')
        mqtt_password: str = os.getenv('MQTT_PASSWORD', '')

        if mqtt_token := os.getenv('MQTT_TOKEN', ''):
            auth = TokenAuth(token=mqtt_token)  # type: ignore
        elif mqtt_username and mqtt_password:
            auth = UserPassAuth(  # type: ignore
                username=mqtt_username,
                password=mqtt_password
            )
        else:
            raise ValueError('No MQTT authentication provided)')

        broker: str = self.get_parameter(
            'mqtt.broker').get_parameter_value().string_value
        port: int = self.get_parameter(
            'mqtt.port').get_parameter_value().integer_value
        timeout: int = self.get_parameter(
            'mqtt.timeout').get_parameter_value().integer_value

        get_logger().info(
            f'Creating MQTT Client Model: {broker = }, {port = }, {timeout = }'
        )

        return MQTTClientModel(
            broker=broker,
            port=port,
            client_id=mqtt_client_id,
            timeout=timeout,
            auth=auth,
            version=CallbackAPIVersion.VERSION2
        )


def main(args=None) -> None:
    load_dotenv()
    create_logger()

    rclpy.init(args=args)

    mqtt_service = MQTTService()
    mqtt_srv_handler = MQTTServiceHandler(mqtt=mqtt_service)
    mqtt_service.connect()

    thread = Thread(target=mqtt_service.spin, daemon=True)

    try:
        get_logger().info('Starting MQTT Service Handler Node')
        thread.start()
        rclpy.spin(mqtt_srv_handler)

    finally:
        terminate(mqtt_service, thread, mqtt_srv_handler)


def terminate(mqtt_service, thread, mqtt_srv_handler) -> None:
    get_logger().info('Shutting down: mqtt_service')
    MQTTService.TERMINATE_CONNECTION = True
    mqtt_service.shutdown()

    get_logger().info('Waiting for thread termination')
    thread.join()

    get_logger().info('Shutting down: mqtt_srv_handler')
    mqtt_srv_handler.destroy_node()

    get_logger().info('Shutting down: rclpy')
    with suppress(RCLError):
        rclpy.shutdown()
