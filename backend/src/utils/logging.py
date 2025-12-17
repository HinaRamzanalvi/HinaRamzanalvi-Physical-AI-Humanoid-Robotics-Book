import logging
import sys
from datetime import datetime
from typing import Any, Dict
import os
from dotenv import load_dotenv

load_dotenv()

# Set up logging configuration
def setup_logging():
    # Get log level from environment, default to INFO
    log_level_str = os.getenv("LOG_LEVEL", "INFO").upper()
    log_level = getattr(logging, log_level_str, logging.INFO)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)

    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)
    root_logger.addHandler(console_handler)

    return root_logger

# Initialize logging
logger = setup_logging()

def log_info(message: str, extra: Dict[str, Any] = None):
    """Log an info message"""
    if extra:
        logger.info(message, extra=extra)
    else:
        logger.info(message)

def log_error(message: str, extra: Dict[str, Any] = None):
    """Log an error message"""
    if extra:
        logger.error(message, extra=extra)
    else:
        logger.error(message)

def log_warning(message: str, extra: Dict[str, Any] = None):
    """Log a warning message"""
    if extra:
        logger.warning(message, extra=extra)
    else:
        logger.warning(message)

def log_debug(message: str, extra: Dict[str, Any] = None):
    """Log a debug message"""
    if extra:
        logger.debug(message, extra=extra)
    else:
        logger.debug(message)