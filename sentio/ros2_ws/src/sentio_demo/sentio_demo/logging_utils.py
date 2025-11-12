"""Logging utilities for demo runner."""

import logging
import json
from datetime import datetime
from pathlib import Path


logger = logging.getLogger(__name__)


def setup_demo_logger(name: str, log_dir: str = '/tmp/sentio_demos') -> logging.Logger:
    """
    Setup logger for demo execution.
    
    Args:
        name: Logger name
        log_dir: Directory for logs
    
    Returns:
        Configured logger
    """
    log_path = Path(log_dir)
    log_path.mkdir(parents=True, exist_ok=True)
    
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(
        logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    )
    logger.addHandler(console_handler)
    
    # File handler
    log_file = log_path / f'{name}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
    file_handler = logging.FileHandler(log_file)
    file_handler.setFormatter(
        logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    )
    logger.addHandler(file_handler)
    
    return logger


def log_step_execution(logger: logging.Logger, step_index: int, step: dict, status: str):
    """Log execution of a choreography step."""
    logger.info(
        f'Step {step_index}: {step.get("action", "unknown")} '
        f'[{status}] - {step.get("description", "")}'
    )


def log_demo_session(logger: logging.Logger, demo_name: str, status: str, duration_s: float):
    """Log demo session completion."""
    logger.info(
        f'Demo "{demo_name}" {status} | Duration: {duration_s:.2f}s'
    )
