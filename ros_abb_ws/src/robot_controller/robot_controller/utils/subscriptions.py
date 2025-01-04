from typing import Any

from robot_controller.utils.mappings import URLMapping
from robot_controller.utils.models import APISubscriptionRequest


def create_subscription() -> APISubscriptionRequest:
    _headers: dict[str, str] = {
        "Accept": "application/xhtml+xml;v=2.0",
        "Content-Type": "application/x-www-form-urlencoded;v=2.0",
    }

    _subscription_request: dict[str, Any] = {
        "resources": []
    }

    _subscriptions: list[str] = [
        URLMapping.speed_ratio.value,
        URLMapping.ctrl_state.value, URLMapping.op_mode.value,
        URLMapping.exec_state.value, URLMapping.exec_cycle.value,
        URLMapping.elog_0.value, URLMapping.elog_1.value,
        URLMapping.elog_2.value, URLMapping.elog_3.value,
        URLMapping.elog_4.value, URLMapping.elog_5.value,
        URLMapping.elog_7.value, URLMapping.elog_8.value,
        URLMapping.elog_9.value, URLMapping.elog_10.value,
        URLMapping.elog_11.value, URLMapping.elog_12.value,
        URLMapping.elog_13.value, URLMapping.elog_15.value,
        URLMapping.elog_17.value, URLMapping.sys_energy.value
    ]

    for index, item in enumerate(_subscriptions):
        _key: str = str(index+1)

        # Adding to resources
        _subscription_request["resources"].append(_key)

        # Adding subscriptions
        _subscription_request[_key] = item
        _subscription_request[f'{_key}-p'] = "1"

    return APISubscriptionRequest(
        api_type="POST",
        api_data=_subscription_request,
        api_url=URLMapping.subscription.value,
        api_headers=_headers,
        api_code=201
    )
