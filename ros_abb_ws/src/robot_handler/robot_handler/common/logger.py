import logging
from logging import Logger

logger: Logger


def create_logger() -> None:
    """
    Creates a logger with the name 'rws_client' and sets the log level.
    Adds a StreamHandler to the logger and configures the formatter
    for log messages.
    """
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
    """
    Returns the logger instance.

    Returns:
        Logger:
            The logger instance.
    """
    global logger

    return logger
