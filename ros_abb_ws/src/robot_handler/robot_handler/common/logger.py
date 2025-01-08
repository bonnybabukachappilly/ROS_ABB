import logging
from logging import Logger

logger: Logger


def create_logger() -> None:
    """Create a logger"""
    global logger

    logger = logging.getLogger('rws_client')
    logger.setLevel(logging.DEBUG)

    consol = logging.StreamHandler()

    logger.addHandler(consol)

    formatter = logging.Formatter(
        fmt="{asctime} | {levelname} | {filename}:{lineno} | {message}",
        style='{',
        datefmt="%d-%b-%y %H:%M:%S"
    )

    consol.setFormatter(formatter)


def get_logger() -> Logger:
    """Returns the logger"""
    global logger

    return logger
