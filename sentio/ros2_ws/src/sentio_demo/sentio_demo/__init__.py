"""
SENTIO Demo Package
===================

Demo runner and choreography engine for coordinated robot performances.

Features:
- YAML-based choreography sequences
- Timed action scheduling
- Multi-modality control (gestures, TTS, LEDs)
- Pause/resume/stop control
- Execution logging and explainability
- Service-based demo control interface

Author: DevSora Deep-Tech Research
License: Proprietary
"""

__version__ = '0.1.0'
__author__ = 'DevSora Team'
__license__ = 'Proprietary'

from .choreography_engine import ChoreographyEngine
from .sequence_parser import SequenceParser
from .demo_controller import DemoController

__all__ = ['ChoreographyEngine', 'SequenceParser', 'DemoController']
