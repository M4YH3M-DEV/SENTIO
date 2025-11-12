"""
AETHER Cognitive AI Stack
==========================

Core AI modules for SENTIO:
- Multimodal fusion (vision + audio)
- Explainability and reasoning trails
- Decision logging and analysis
- Temporal smoothing and filtering

Author: DevSora Deep-Tech Research
License: Proprietary
"""

__version__ = '0.1.0'
__author__ = 'DevSora Team'
__license__ = 'Proprietary'

import os
import logging

# Configure AETHER logging
log_dir = os.path.join(os.path.dirname(__file__), 'logs')
os.makedirs(log_dir, exist_ok=True)

logging.getLogger('aether').setLevel(logging.DEBUG)

from .fusion import FusionEngine
from .explainability import ReasoningTrail, DecisionLogger

__all__ = ['FusionEngine', 'ReasoningTrail', 'DecisionLogger']
