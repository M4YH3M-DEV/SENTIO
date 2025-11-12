"""
Voice Configuration for Piper TTS

Manages voice profiles, language settings, and synthesis parameters.
"""

from typing import Dict, Optional
from dataclasses import dataclass
import logging


logger = logging.getLogger(__name__)


@dataclass
class VoiceProfile:
    """Represents a Piper voice profile."""
    
    name: str
    language: str
    gender: str  # 'male' or 'female'
    emotion: str  # 'neutral', 'happy', 'sad', etc.
    model_key: str  # Internal model identifier
    
    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'name': self.name,
            'language': self.language,
            'gender': self.gender,
            'emotion': self.emotion,
            'model_key': self.model_key,
        }


class VoiceConfig:
    """
    Manages Piper voice profiles and voice selection logic.
    """
    
    # Default voice profiles (can be extended)
    DEFAULT_PROFILES = {
        'sentio_neutral': VoiceProfile(
            name='sentio_neutral',
            language='en-US',
            gender='neutral',
            emotion='neutral',
            model_key='en_US-lessac-medium',
        ),
        'sentio_friendly': VoiceProfile(
            name='sentio_friendly',
            language='en-US',
            gender='neutral',
            emotion='happy',
            model_key='en_US-lessac-medium',
        ),
        'sentio_concerned': VoiceProfile(
            name='sentio_concerned',
            language='en-US',
            gender='neutral',
            emotion='sad',
            model_key='en_US-lessac-medium',
        ),
        'default': VoiceProfile(
            name='default',
            language='en-US',
            gender='neutral',
            emotion='neutral',
            model_key='en_US-lessac-medium',
        ),
    }
    
    def __init__(self):
        """Initialize voice configuration."""
        self.profiles = dict(self.DEFAULT_PROFILES)
        self.current_voice = 'default'
        logger.info(f'VoiceConfig initialized with {len(self.profiles)} profiles')
    
    def get_profile(self, voice_name: str) -> Optional[VoiceProfile]:
        """
        Get a voice profile by name.
        
        Args:
            voice_name: Name of voice profile
        
        Returns:
            VoiceProfile or None if not found
        """
        return self.profiles.get(voice_name)
    
    def get_current_profile(self) -> VoiceProfile:
        """Get currently active voice profile."""
        return self.profiles[self.current_voice]
    
    def set_current_voice(self, voice_name: str) -> bool:
        """
        Set the current voice profile.
        
        Args:
            voice_name: Name of voice to activate
        
        Returns:
            True if successful, False if voice not found
        """
        if voice_name not in self.profiles:
            logger.warning(f'Voice profile not found: {voice_name}')
            return False
        
        self.current_voice = voice_name
        logger.info(f'Voice switched to: {voice_name}')
        return True
    
    def add_profile(self, profile: VoiceProfile) -> bool:
        """
        Register a new voice profile.
        
        Args:
            profile: VoiceProfile to add
        
        Returns:
            True if added, False if already exists
        """
        if profile.name in self.profiles:
            logger.warning(f'Profile already exists: {profile.name}')
            return False
        
        self.profiles[profile.name] = profile
        logger.info(f'Voice profile registered: {profile.name}')
        return True
    
    def list_voices(self) -> Dict[str, Dict]:
        """Get all available voices as dict."""
        return {name: profile.to_dict() for name, profile in self.profiles.items()}
    
    def select_emotion_voice(self, emotion: str) -> Optional[str]:
        """
        Select a voice matching an emotion.
        
        Args:
            emotion: Emotion keyword ('happy', 'sad', 'neutral', etc.)
        
        Returns:
            Voice name if found, None otherwise
        """
        # Map emotion keywords to voice names
        emotion_map = {
            'happy': 'sentio_friendly',
            'positive': 'sentio_friendly',
            'excited': 'sentio_friendly',
            'sad': 'sentio_concerned',
            'angry': 'sentio_concerned',
            'fearful': 'sentio_concerned',
            'neutral': 'sentio_neutral',
            'surprised': 'sentio_friendly',
        }
        
        voice_name = emotion_map.get(emotion.lower())
        
        if voice_name and self.set_current_voice(voice_name):
            return voice_name
        
        return None
    
    def get_synthesis_params(self) -> Dict:
        """
        Get Piper synthesis parameters for current voice.
        
        Returns:
            Dictionary of synthesis parameters
        """
        profile = self.get_current_profile()
        
        return {
            'speaker': profile.model_key,
            'length_scale': 1.0,  # Speech speed (1.0 = normal)
            'noise_scale': 0.667,  # Variability
            'noise_w': 0.8,  # Phoneme duration variability
        }
