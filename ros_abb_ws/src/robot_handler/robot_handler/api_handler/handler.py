from collections.abc import Callable
from typing import Any

from requests import Response, Session

from robot_handler.api_handler.models import APIConnection, APIRequest
from robot_handler.common.logger import get_logger


class APIClient:
    SESSION = Session()

    __slots__: list[str] = ['__api', '__header']

    def __init__(self, api: APIConnection) -> None:
        self.__api: APIConnection = api
        self.__header: dict[str, str]

    @property
    def header(self) -> dict[str, str]:
        return self.__header

    @header.setter
    def header(self, header: dict[str, str]) -> None:
        self.__header = header

    def __get(
        self, path: str, _: Any,
            headers: dict[str, str] | None = None) -> Response:

        _headers: dict[str, str] = headers or self.header

        return self.__class__.SESSION.get(
            url=self.__api.host + path,
            auth=self.__api.auth,
            headers=_headers
        )

    def __post(
            self, path: str, data: dict[str, Any],
            headers: dict[str, str] | None = None) -> Response:

        _headers: dict[str, str] = headers or self.header

        return self.__class__.SESSION.post(
            url=self.__api.host + path,
            auth=self.__api.auth,
            headers=_headers,
            data=data
        )

    def __delete(
            self,  path: str,  _: Any,
            headers: dict[str, str] | None = None) -> Response:

        _headers: dict[str, str] = headers or self.header

        return self.__class__.SESSION.delete(
            url=self.__api.host + path,
            auth=self.__api.auth,
            headers=_headers
        )

    def __get_request_type(
            self,
            request: str) -> Callable[
                [str, Any, dict[str, str] | None], Response]:

        match request:
            case 'GET':
                return self.__get
            case 'POST':
                return self.__post
            case 'DELETE':
                return self.__delete
            case _:
                raise ValueError('Invalid Request Type')

    def process_api_request(self, request: APIRequest) -> Response | None:
        _request: Callable[
            [str, Any, dict[str, str] | None],
            Response] = self.__get_request_type(request.api_type)

        get_logger().debug(f'Sending: {request.api_type} | {request.api_url}')

        _response: Response = _request(
            request.api_url, request.api_data, request.api_headers)

        get_logger().debug(f'Response Code: {_response.status_code}')

        if _response.status_code == request.expected_code:
            return _response

        get_logger().debug(f'Response: {_response.text}')

        get_logger().warning(
            f'Invalid status code received: {_response.status_code}')
        return None
