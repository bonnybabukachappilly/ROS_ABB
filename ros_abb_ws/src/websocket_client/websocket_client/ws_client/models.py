from typing import Any, NamedTuple


class WSConnection(NamedTuple):
    """Websocket connection model"""
    url: str
    header: dict[str, Any]
    protocol: list[str]
