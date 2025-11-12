"""
Logging Utilities for Policy Engine

Structured logging with JSON output support.
"""

import logging
import json
from typing import Dict, Any
from datetime import datetime


logger = logging.getLogger(__name__)


class JSONFormatter(logging.Formatter):
    """Format logs as JSON for structured logging."""
    
    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON."""
        log_data = {
            'timestamp': datetime.fromtimestamp(record.created).isoformat(),
            'level': record.levelname,
            'logger': record.name,
            'message': record.getMessage(),
            'module': record.module,
        }
        
        # Add extra fields if present
        if hasattr(record, 'extra_data'):
            log_data.update(record.extra_data)
        
        return json.dumps(log_data)


def setup_policy_logger(name: str, json_format: bool = False) -> logging.Logger:
    """
    Setup logger for policy engine.
    
    Args:
        name: Logger name
        json_format: Use JSON formatting if True
    
    Returns:
        Configured logger
    """
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    
    # Console handler
    handler = logging.StreamHandler()
    
    if json_format:
        formatter = JSONFormatter()
    else:
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    
    return logger


def log_policy_decision(
    logger: logging.Logger,
    decision_type: str,
    inputs: Dict[str, Any],
    decision: Dict[str, Any],
    confidence: float = 1.0
):
    """
    Log a policy decision with context.
    
    Args:
        logger: Logger instance
        decision_type: Type of decision
        inputs: Input values
        decision: Decision made
        confidence: Confidence in decision (0-1)
    """
    extra_data = {
        'decision_type': decision_type,
        'inputs': inputs,
        'decision': decision,
        'confidence': confidence
    }
    
    record = logging.LogRecord(
        name=logger.name,
        level=logging.INFO,
        pathname='',
        lineno=0,
        msg=f'{decision_type} decision: {decision}',
        args=(),
        exc_info=None
    )
    record.extra_data = extra_data
    
    logger.handle(record)
