"""
AETHER Fusion Module

Multimodal affect fusion combining:
- Vision-based emotion (facial expression)
- Audio-based emotion (speech tone)
- Temporal consistency and smoothing
- Uncertainty estimation

Author: DevSora Deep-Tech Research
License: Proprietary
"""

from .fusion_engine import FusionEngine
from .fuse_affect import fuse_affect
from .weighting_strategy import WeightingStrategy

__all__ = ['FusionEngine', 'fuse_affect', 'WeightingStrategy']
