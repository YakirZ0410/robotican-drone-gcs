"""Console and size-bounded UTF-8 log files for the drone process."""

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path


DRONE_LOGGER_NAME = "drone_gcs.drone"
DEFAULT_MAX_BYTES = 5_000_000
DEFAULT_BACKUP_COUNT = 3


def close_drone_logging() -> None:
    """Flush and release handlers owned by the simulator entry point."""
    logger = logging.getLogger(DRONE_LOGGER_NAME)
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
        handler.close()
    logger.setLevel(logging.NOTSET)
    logger.propagate = True


def configure_drone_logging(
    log_file: str | Path,
    level: str = "INFO",
    *,
    max_bytes: int = DEFAULT_MAX_BYTES,
    backup_count: int = DEFAULT_BACKUP_COUNT,
) -> logging.Logger:
    """Configure this process's logger, falling back to console on file errors."""
    if max_bytes <= 0 or backup_count < 1:
        raise ValueError("Log size and backup count must be positive")

    close_drone_logging()
    logger = logging.getLogger(DRONE_LOGGER_NAME)
    logger.setLevel(level)
    logger.propagate = False
    formatter = logging.Formatter(
        "%(asctime)s %(levelname)-7s [%(process)d] %(message)s"
    )
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    logger.addHandler(console)

    path = Path(log_file)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        file_handler = RotatingFileHandler(
            path, maxBytes=max_bytes, backupCount=backup_count, encoding="utf-8",
        )
    except OSError as exc:
        logger.warning("Could not open log file %s; console logging only: %s", path, exc)
    else:
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger
