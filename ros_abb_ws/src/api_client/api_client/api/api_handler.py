from typing import Any

from requests import Response

from api_client.api.models import APIConnection


class APIHandler:
    """
    Handles API requests and provides methods for making
    GET, POST, and DELETE requests.

    Args:
        api (APIConnection):
            An instance of APIConnection.

    Attributes:
        __api (APIConnection):
            The APIConnection instance.
        __header (dict[str, str]):
            The header for API requests.

    Properties:
        header (dict[str, str]):
            The header for API requests.

    Methods:
        get(
            path: str,
            headers: dict[str, str] | None = None) -> Response:
            Sends a GET request to the specified path.

        post(
            path: str,
            data: dict[str, Any],
            headers: dict[str, str] | None = None) -> Response:
            Sends a POST request to the specified path with the given data.

        delete(
            path: str,
            headers: dict[str, str] | None = None) -> Response:
            Sends a DELETE request to the specified path.

    """

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

    def get(
            self,
            path: str,
            _: Any,
            headers: dict[str, str] | None = None) -> Response:
        """
        Sends a GET request to the specified path.

        Args:
            path (str): The path for the GET request.
            headers (dict[str, str], optional): Custom headers for the
                request. Defaults to None.

        Returns:
            Response: The response object.

        """
        _headers: dict[str, str] = headers or self.header

        return self.__api.session.get(
            url=self.__api.host + path,
            auth=self.__api.auth,
            headers=_headers
        )

    def post(
            self,
            path: str,
            data: dict[str, Any],
            headers: dict[str, str] | None = None) -> Response:
        """
        Sends a POST request to the specified path with the given data.

        Args:
            path (str): The path for the POST request.
            data (dict[str, Any]): The data to be sent with the request.
            headers (dict[str, str], optional): Custom headers for the
                request. Defaults to None.

        Returns:
            Response: The response object.

        """
        _headers: dict[str, str] = headers or self.header

        return self.__api.session.post(
            url=self.__api.host + path,
            auth=self.__api.auth,
            headers=_headers,
            data=data
        )

    def delete(
            self,
            path: str,
            _: Any,
            headers: dict[str, str] | None = None) -> Response:
        """
        Sends a DELETE request to the specified path.

        Args:
            path (str): The path for the DELETE request.
            headers (dict[str, str], optional): Custom headers for the
                request. Defaults to None.

        Returns:
            Response: The response object.

        """
        _headers: dict[str, str] = headers or self.header

        return self.__api.session.delete(
            url=self.__api.host + path,
            auth=self.__api.auth,
            headers=_headers
        )
