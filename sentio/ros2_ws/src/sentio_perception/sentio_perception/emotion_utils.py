"""
Emotion Utility Functions

Maps emotion classifications to valence-arousal coordinates and categorical labels.
"""

import logging
from typing import Dict, Tuple, Optional
from enum import IntEnum


logger = logging.getLogger(__name__)


class EmotionLabel(IntEnum):
    """Categorical emotion labels."""
    NEUTRAL = 0
    HAPPY = 1
    SAD = 2
    ANGRY = 3
    FEARFUL = 4
    DISGUSTED = 5
    SURPRISED = 6


class EmotionMapper:
    """
    Maps emotion classifications to standardized representation.
    
    Uses valence-arousal model:
    - Valence: -1.0 (negative) to +1.0 (positive)
    - Arousal: 0.0 (calm) to 1.0 (excited)
    """
    
    # Emotion to valence-arousal mapping
    EMOTION_VA_MAP = {
        'neutral': (0.0, 0.3),
        'happy': (0.8, 0.6),
        'sad': (-0.7, 0.2),
        'angry': (-0.8, 0.8),
        'fearful': (-0.5, 0.9),
        'disgusted': (-0.7, 0.4),
        'surprised': (0.5, 0.9),
    }
    
    # Emotion to categorical label
    EMOTION_LABEL_MAP = {
        'neutral': EmotionLabel.NEUTRAL,
        'happy': EmotionLabel.HAPPY,
        'sad': EmotionLabel.SAD,
        'angry': EmotionLabel.ANGRY,
        'fearful': EmotionLabel.FEARFUL,
        'disgusted': EmotionLabel.DISGUSTED,
        'surprised': EmotionLabel.SURPRISED,
    }
    
    # Reverse mapping
    LABEL_EMOTION_MAP = {v: k for k, v in EMOTION_LABEL_MAP.items()}
    
    @staticmethod
    def emotion_to_valence_arousal(
        emotion: str,
        confidence: float = 1.0
    ) -> Tuple[float, float]:
        """
        Map emotion label to valence-arousal coordinates.
        
        Args:
            emotion: Emotion name (e.g., 'happy', 'sad')
            confidence: Confidence score (0-1) to scale arousal
        
        Returns:
            Tuple of (valence, arousal)
        """
        emotion_lower = emotion.lower().strip()
        
        if emotion_lower not in EmotionMapper.EMOTION_VA_MAP:
            logger.warning(f'Unknown emotion: {emotion_lower}, using neutral')
            valence, arousal = EmotionMapper.EMOTION_VA_MAP['neutral']
        else:
            valence, arousal = EmotionMapper.EMOTION_VA_MAP[emotion_lower]
        
        # Scale arousal by confidence
        arousal = arousal * confidence
        
        return float(valence), float(arousal)
    
    @staticmethod
    def emotion_to_label(emotion: str) -> int:
        """
        Map emotion name to integer label.
        
        Args:
            emotion: Emotion name
        
        Returns:
            Integer label
        """
        emotion_lower = emotion.lower().strip()
        return int(EmotionMapper.EMOTION_LABEL_MAP.get(
            emotion_lower,
            EmotionLabel.NEUTRAL
        ))
    
    @staticmethod
    def label_to_emotion(label: int) -> str:
        """
        Map integer label to emotion name.
        
        Args:
            label: Integer emotion label
        
        Returns:
            Emotion name string
        """
        return EmotionMapper.LABEL_EMOTION_MAP.get(label, 'neutral')
    
    @staticmethod
    def va_to_emotion(valence: float, arousal: float) -> str:
        """
        Approximate closest emotion from valence-arousal coordinates.
        
        Args:
            valence: Valence (-1 to +1)
            arousal: Arousal (0 to 1)
        
        Returns:
            Closest emotion name
        """
        min_dist = float('inf')
        closest_emotion = 'neutral'
        
        for emotion, (va_val, va_arou) in EmotionMapper.EMOTION_VA_MAP.items():
            # Euclidean distance in VA space
            dist = ((valence - va_val) ** 2 + (arousal - va_arou) ** 2) ** 0.5
            
            if dist < min_dist:
                min_dist = dist
                closest_emotion = emotion
        
        return closest_emotion
    
    @staticmethod
    def get_emotion_intensity(arousal: float) -> str:
        """
        Get intensity level based on arousal.
        
        Args:
            arousal: Arousal value (0-1)
        
        Returns:
            Intensity: 'calm', 'moderate', 'intense'
        """
        if arousal < 0.33:
            return 'calm'
        elif arousal < 0.66:
            return 'moderate'
        else:
            return 'intense'
    
    @staticmethod
    def normalize_confidence(raw_confidence: float) -> float:
        """
        Normalize confidence score to 0-1 range.
        
        Args:
            raw_confidence: Raw confidence value
        
        Returns:
            Normalized confidence (0-1)
        """
        return max(0.0, min(1.0, raw_confidence))


class SmoothingFilter:
    """
    Exponential moving average filter for temporal smoothing.
    
    Reduces jitter in emotion predictions over time.
    """
    
    def __init__(self, alpha: float = 0.3):
        """
        Initialize smoothing filter.
        
        Args:
            alpha: Smoothing factor (0-1). Higher = more responsive.
        """
        self.alpha = alpha
        self.prev_valence: Optional[float] = None
        self.prev_arousal: Optional[float] = None
    
    def smooth(self, valence: float, arousal: float) -> Tuple[float, float]:
        """
        Apply exponential smoothing to emotion values.
        
        Args:
            valence: Current valence
            arousal: Current arousal
        
        Returns:
            Tuple of smoothed (valence, arousal)
        """
        if self.prev_valence is None:
            self.prev_valence = valence
            self.prev_arousal = arousal
            return valence, arousal
        
        # EMA formula: smoothed = alpha * current + (1-alpha) * previous
        smoothed_valence = (self.alpha * valence + 
                           (1 - self.alpha) * self.prev_valence)
        smoothed_arousal = (self.alpha * arousal + 
                           (1 - self.alpha) * self.prev_arousal)
        
        self.prev_valence = smoothed_valence
        self.prev_arousal = smoothed_arousal
        
        return smoothed_valence, smoothed_arousal
    
    def reset(self):
        """Reset filter state."""
        self.prev_valence = None
        self.prev_arousal = None
