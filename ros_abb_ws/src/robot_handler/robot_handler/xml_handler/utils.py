import json
from typing import Any
from xml.etree.ElementTree import Element

from requests.models import Response

from robot_handler.api_handler.models import APIClientModel, APIRequest
from robot_handler.common.mappings import (
    ROBOT_ELOG_KEYS,
    ROBOT_ELOG_TYPE,
    ROBOT_ENERGY,
)


class ParserCollections:

    def __init__(self, api: APIClientModel) -> None:
        self.__api: APIClientModel = api

    def __format_data(
            self, response: Response | None,
            model: dict[str, str]) -> dict[str, Any] | None:

        if response is None:
            return None

        decoded: Any = json.loads(response.text)
        data: dict[str, str] | None = decoded.get('state')[0]

        if data is None:
            return None

        processed_data: dict[str, Any] = {
            value: data.get(key) for key, value in model.items()
        }
        return processed_data

    def state_parser(
            self, tag: str, et: Element,
            ns: dict[str, str]) -> dict[str, Any] | None:

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
            self, tag: str, et: Element,
            ns: dict[str, str]) -> dict[str, Any] | None:

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

        data: dict[str, Any] | None = self.__format_data(
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
        del ns

        request = APIRequest(
            api_type='GET',
            api_data=None,
            api_url="/rw/system/energy/",
            api_headers=None,
            expected_code=200
        )

        return self.__format_data(
            self.__api.process_api_request(request),
            model=ROBOT_ENERGY
        )
