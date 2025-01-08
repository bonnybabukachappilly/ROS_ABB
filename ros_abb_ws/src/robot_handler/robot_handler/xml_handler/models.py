from collections.abc import Callable
from typing import Any
from xml.etree.ElementTree import Element

XMLDataParser = Callable[[Element, dict[str, str]], dict[str, Any] | None]
