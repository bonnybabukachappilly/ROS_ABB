from rclpy.node import Any

from robot_handler.api_handler.models import APIRequest


def create_subscription(subscriptions: list[str], url: str) -> APIRequest:
    _headers: dict[str, str] = {
        "Accept": "application/xhtml+xml;v=2.0",
        "Content-Type": "application/x-www-form-urlencoded;v=2.0",
    }

    _subscription_request: dict[str, Any] = {
        "resources": []
    }

    for index, item in enumerate(subscriptions):
        _key: str = str(index+1)

        # Adding to resources
        _subscription_request["resources"].append(_key)

        # Adding subscriptions
        _subscription_request[_key] = item
        _subscription_request[f'{_key}-p'] = "1"

    return APIRequest(
        api_type='POST',
        api_data=_subscription_request,
        api_url=url,
        api_headers=_headers,
        expected_code=201
    )
