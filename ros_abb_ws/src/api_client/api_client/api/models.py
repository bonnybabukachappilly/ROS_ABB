from typing import NamedTuple

from requests import Session
from requests.auth import HTTPBasicAuth, HTTPDigestAuth


class APIConnection(NamedTuple):
    """
    Represents a connection to an API.

    Attributes:
        host (str): The host URL of the API.
        session (Session): The requests session object used for
            making API requests.
        auth (HTTPDigestAuth | HTTPBasicAuth): The authentication method
            used for the API connection.
    """
    host: str
    session: Session
    auth: HTTPDigestAuth | HTTPBasicAuth
