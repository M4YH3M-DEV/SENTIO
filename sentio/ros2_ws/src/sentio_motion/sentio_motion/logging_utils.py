"""Logging utilities for motion control."""

import logging
import json
from datetime import datetime


logger = logging.getLogger(__name__)


def setup_motion_logger(name: str) -> logging.Logger:
    """Setup logger for motion module."""
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    
    handler = logging.StreamHandler()
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    
    return logger


def log_motion_command(logger: logging.Logger, command: dict, target: str = ''):
    """Log a motion command."""
    logger.info(
        f'Motion command: {target} | Servos: {len(command.get("positions", {}))} | '
        f'Duration: {command.get("duration_s", 0):.2f}s'
    )


def log_safety_event(logger: logging.Logger, event_type: str, details: str):
    """Log a safety-related event."""
    logger.warning(f'Safety event [{event_type}]: {details}')
