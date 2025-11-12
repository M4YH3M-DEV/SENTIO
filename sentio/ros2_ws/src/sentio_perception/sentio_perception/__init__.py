"""
SENTIO Perception Package
==========================

Multimodal emotion perception from facial expressions and audio tone.

Features:
- Real-time face detection and expression classification
- Audio affect analysis from speech tone
- Group detection and recognition (multi-person)
- ONNX model inference on CPU/GPU
- Emotion mapping to valence-arousal space
- Explainability and confidence reporting

Author: DevSora Deep-Tech Research
License: Proprietary
"""

__version__ = '0.1.0'
__author__ = 'DevSora Team'
__license__ = 'Proprietary'

from .model_loader import ModelLoader
from .emotion_utils import EmotionMapper

__all__ = ['ModelLoader', 'EmotionMapper']
