"""
SENTIO Text-to-Speech Bridge Package
=====================================

Integrates Piper TTS engine with ROS 2 for SENTIO voice synthesis.

Features:
- HTTP client for Piper TTS server
- Audio file management and caching
- Voice profile selection
- Asynchronous TTS processing
- Error handling and retry logic
- Status and diagnostics publishing

Author: DevSora Deep-Tech Research
License: Proprietary
"""

__version__ = '0.1.0'
__author__ = 'DevSora Team'
__license__ = 'Proprietary'

from .piper_client import PiperClient
from .audio_manager import AudioManager
from .voice_config import VoiceConfig

__all__ = ['PiperClient', 'AudioManager', 'VoiceConfig']
