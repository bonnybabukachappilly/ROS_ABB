from typing import Any, NamedTuple, Protocol

from requests import Response, Session
from requests.auth import HTTPBasicAuth, HTTPDigestAuth


class APIRequest(NamedTuple):
    api_type: str
    api_data: dict[str, Any] | None
    api_url: str
    api_headers: dict[str, str] | None
    expected_code: int


class APIConnection(NamedTuple):
    host: str
    auth: HTTPDigestAuth | HTTPBasicAuth


class APIClientModel(Protocol):
    SESSION: Session

    __slots__: list[str] = ['__api', '__header']

    def __init__(self, api: APIConnection) -> None:
        ...

    def process_api_request(self, request: APIRequest) -> Response | None:
        ...
