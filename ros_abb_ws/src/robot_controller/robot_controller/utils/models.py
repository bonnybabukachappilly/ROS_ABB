from typing import Any, NamedTuple


class APISubscriptionRequest(NamedTuple):
    api_type: str
    api_request: dict[str, Any]
    api_headers: dict[str, str]
