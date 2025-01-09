from typing import Any
from xml.etree.ElementTree import Element

from robot_handler.api_handler.models import APIClientModel, APIRequest
from robot_handler.api_handler.utils import format_api_model
from robot_handler.common.mappings import (
    ROBOT_ELOG_KEYS,
    ROBOT_ELOG_TYPE,
    ROBOT_ENERGY,
)


class ParserCollections:
    """
    A collection of XML parsing utility methods.

    Args:
        api (APIClientModel):
            An instance of the APIClientModel class.

    Attributes:
        __api (APIClientModel):
            An instance of the APIClientModel class.

    """

    def __init__(self, api: APIClientModel) -> None:
        """
        Initializes a new instance of the Utils class.

        Args:
            api (APIClientModel):
                An instance of the APIClientModel class.
        """
        self.__api: APIClientModel = api

    def state_parser(
        self, tag: str, et: Element, ns: dict[str, str]
    ) -> dict[str, Any] | None:
        """
        Parses the XML element and extracts the value of the specified tag.

        Args:
            tag (str):
                The tag to search for within the XML element.
            et (Element):
                The XML element to parse.
            ns (dict[str, str]):
                The namespace dictionary for XML element.

        Returns:
            dict[str, Any] | None:
                A dictionary containing the tag as the key and its value,
                or None if the tag is not found.

        """
        span_element: list[Element] = et.findall(".//xhtml:span", ns)
        for element in span_element:
            if element is not None:
                span_class: str | None = element.get("class")
                if span_class != tag:
                    continue
                span_value: str | None = element.text
                if span_class and span_value:
                    return {span_class: span_value}
        return None

    def elog_parser(
        self, tag: str, et: Element, ns: dict[str, str]
    ) -> dict[str, Any] | None:
        """
        Parses the elog XML element and returns the parsed data as a
        dictionary.

        Args:
            tag (str):
                The tag name of the elog XML element.
            et (Element):
                The elog XML element to be parsed.
            ns (dict[str, str]):
                The namespace dictionary for the XML element.

        Returns:
            dict[str, Any] | None:
                The parsed data as a dictionary, or None if parsing fails.
        """
        _id: dict[str, Any] | None = self.state_parser(tag=tag, et=et, ns=ns)

        if _id is None:
            return None

        request = APIRequest(
            api_type='GET',
            api_data=None,
            api_url=f"/rw/elog/0/{_id.get(tag)}?lang=eng",
            api_headers=None,
            expected_code=200
        )

        data: dict[str, Any] | None = format_api_model(
            self.__api.process_api_request(request),
            model=ROBOT_ELOG_KEYS
        )

        if data is not None:
            log_type: str = ROBOT_ELOG_TYPE[data['elog_msgtype']]
            data['elog_msgtype'] = log_type
            return data

        return None

    def energy_parser(
            self, _: Element,
            ns: dict[str, str]) -> dict[str, Any] | None:
        """
        Parses the energy data from the API response and formats it.

        Args:
            _:
                The XML element (not used in this method).
            ns:
                A dictionary containing namespace mappings
                (not used in this method).

        Returns:
            A dictionary containing the formatted energy data,
            or None if the API request fails.
        """
        del ns

        request = APIRequest(
            api_type='GET',
            api_data=None,
            api_url="/rw/system/energy/",
            api_headers=None,
            expected_code=200
        )

        return format_api_model(
            self.__api.process_api_request(request),
            model=ROBOT_ENERGY
        )
