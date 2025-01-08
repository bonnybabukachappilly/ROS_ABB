import json
import xml.etree.ElementTree as ET
from functools import partial
from typing import Any
from xml.etree.ElementTree import Element

from robot_handler.api_handler.models import APIClientModel
from robot_handler.xml_handler.models import XMLDataParser
from robot_handler.xml_handler.utils import ParserCollections


class XMLHandler:
    """
    A class that handles XML parsing and data extraction.

    Attributes:
        __api (APIClientModel):
            The API client model.
        __namespace (dict[str, str]):
            The XML namespace.
        __parser (ParserCollections):
            The parser collections.

    Methods:
        __init__(self, api: APIClientModel) -> None:
            Initializes the XMLHandler instance.
        __parsers(self, key: str) -> XMLDataParser | None:
            Returns the corresponding parser based on the given key.
        parse(self, xml: str) -> str:
            Parses the XML string and returns the extracted data as a
            JSON string.
    """

    __slots__: list[str] = ["__api", "__namespace", "__parser"]

    def __init__(self, api: APIClientModel) -> None:
        """
        Initializes the XMLHandler instance.

        Args:
            api (APIClientModel): The API client model.
        """
        self.__namespace: dict[str, str] = {
            "xhtml": "http://www.w3.org/1999/xhtml"}

        self.__parser = ParserCollections(api)

    def __parsers(self, key: str) -> XMLDataParser | None:
        """
        Returns the corresponding parser based on the given key.

        Args:
            key (str):
                The key to identify the parser.

        Returns:
            XMLDataParser | None:
                The corresponding parser or None if not found.
        """
        _state_parser: Any = self.__parser.state_parser

        return {
            "opmode": partial(_state_parser, "opmode"),
            "speedratio": partial(_state_parser, "speedratio"),
            "ctrlstate": partial(_state_parser, "ctrlstate"),
            "ctrlexecstate": partial(_state_parser, "ctrlexecstate"),
            "seqnum": partial(self.__parser.elog_parser, "seqnum"),
            "rapidexeccycle": partial(_state_parser, "rapidexeccycle"),
            "energy-state": partial(self.__parser.energy_parser),
        }.get(key)

    def parse(self, xml: str) -> str:
        """
        Parses the XML string and returns the extracted data as a JSON string.

        Args:
            xml (str): The XML string to parse.

        Returns:
            str: The extracted data as a JSON string.
        """
        root: Element = ET.fromstring(xml)
        span: list[Element] = root.findall(".//xhtml:span", self.__namespace)
        key: str | None = None

        data: dict[str, Any] = {}

        for element in span:

            if element is not None:
                key = element.get("class")

            if key is None:
                return ""

            parser: XMLDataParser | None = self.__parsers(key=key)

            if parser is None:
                return ""

            parsed_data: dict[str, Any] | None = parser(root, self.__namespace)

            if parsed_data is not None:
                data = data | parsed_data

        return json.dumps(data)
