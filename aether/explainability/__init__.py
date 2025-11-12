"""
AETHER Explainability Module

Provides transparency into AI decisions through:
- Decision reasoning trails
- Confidence attribution
- Feature importance
- Decision logging

Author: DevSora Deep-Tech Research
License: Proprietary
"""

from .reasoning_trail import ReasoningTrail
from .decision_logger import DecisionLogger
from .explanation_formatter import ExplanationFormatter

__all__ = ['ReasoningTrail', 'DecisionLogger', 'ExplanationFormatter']
