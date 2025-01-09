import json
from typing import Any

from requests.models import Response


def format_api_model(
        response: Response | None,
        model: dict[str, str]) -> dict[str, Any] | None:
    """
    Formats the API model based on the provided response and model.

    Args:
        response (Response | None):
            The API response object.
        model (dict[str, str]):
            The model dictionary containing the keys and values to extract
            from the response.

    Returns:
        dict[str, Any] | None:
            The processed data dictionary or None if the
            response or data is None.
    """

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
