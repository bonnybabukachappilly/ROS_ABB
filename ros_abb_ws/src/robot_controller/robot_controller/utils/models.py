from typing import Any, NamedTuple


class APISubscriptionRequest(NamedTuple):
    api_type: str
    api_data: dict[str, Any] | None
    api_url: str
    api_headers: dict[str, str]
    api_code: int


class WSConnection(NamedTuple):
    """Websocket connection model"""
    url: str
    header: dict[str, Any]
    protocol: list[str]