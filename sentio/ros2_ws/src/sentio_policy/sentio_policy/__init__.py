"""
SENTIO Policy Engine Package
============================

Decision-making layer mapping affect to behaviors.

Features:
- Rule-based behavior decision logic
- Emotion-to-action mapping
- Context-aware behavior selection
- Validation and safety checks
- Comprehensive logging

Author: DevSora Deep-Tech Research
License: Proprietary
"""

__version__ = '0.1.0'
__author__ = 'DevSora Team'
__license__ = 'Proprietary'

from .rule_evaluator import RuleEvaluator
from .behavior_mapper import BehaviorMapper
from .validators import BehaviorValidator

__all__ = ['RuleEvaluator', 'BehaviorMapper', 'BehaviorValidator']
