"""
Behavior Mapper

Maps emotions and context to robot behaviors.
"""

import logging
from typing import Dict, Optional, List
from enum import Enum


logger = logging.getLogger(__name__)


class BehaviorIntensity(Enum):
    """Behavior intensity levels."""
    SUBTLE = 'subtle'
    NORMAL = 'normal'
    STRONG = 'strong'


class BehaviorMapper:
    """
    Maps affect and context to robot behaviors.
    
    Supports:
    - Emotion-based behaviors
    - Context-aware adjustments
    - Intensity modulation
    - Multi-modality output (gesture, LED, TTS)
    """
    
    # Base behavior mappings
    EMOTION_BEHAVIORS = {
        'happy': {
            'gesture': 'happy_bounce',
            'led': {'color': 'yellow', 'pattern': 'pulse'},
            'tts': {'text': 'I feel great!', 'emotion': 'happy'},
        },
        'sad': {
            'gesture': 'lean_back',
            'led': {'color': 'blue', 'pattern': 'slow_pulse'},
            'tts': {'text': 'I feel a bit down...', 'emotion': 'sad'},
        },
        'angry': {
            'gesture': 'shake_head',
            'led': {'color': 'red', 'pattern': 'blink'},
            'tts': {'text': 'This is frustrating!', 'emotion': 'angry'},
        },
        'fearful': {
            'gesture': 'lean_back',
            'led': {'color': 'orange', 'pattern': 'pulse'},
            'tts': {'text': 'I am uncertain...', 'emotion': 'fearful'},
        },
        'disgusted': {
            'gesture': 'shake_head',
            'led': {'color': 'green', 'pattern': 'steady'},
            'tts': {'text': 'That is not good.', 'emotion': 'disgusted'},
        },
        'surprised': {
            'gesture': 'nod',
            'led': {'color': 'cyan', 'pattern': 'blink'},
            'tts': {'text': 'Wow, that surprised me!', 'emotion': 'surprised'},
        },
        'neutral': {
            'gesture': 'idle',
            'led': {'color': 'white', 'pattern': 'steady'},
            'tts': {'text': '', 'emotion': 'neutral'},
        },
    }
    
    # Context modifiers
    CONTEXT_MODIFIERS = {
        'group_size': {
            'high': {
                'gesture_modifier': 'exaggerate',
                'led_modifier': 'increase_intensity',
                'tts_modifier': 'louder'
            },
            'low': {
                'gesture_modifier': 'subtle',
                'led_modifier': 'dim',
                'tts_modifier': 'normal'
            }
        },
        'proximity': {
            'close': {
                'gesture_modifier': 'gentle',
                'tts_modifier': 'softer'
            },
            'far': {
                'gesture_modifier': 'exaggerated',
                'tts_modifier': 'louder'
            }
        }
    }
    
    def __init__(self):
        """Initialize behavior mapper."""
        pass
    
    def map_emotion_to_behavior(
        self,
        emotion_label: str,
        valence: float,
        arousal: float,
        confidence: float = 1.0,
        context: Optional[Dict] = None
    ) -> Dict:
        """
        Map emotion to behavior.
        
        Args:
            emotion_label: Emotion name (e.g., 'happy')
            valence: Valence score (-1 to +1)
            arousal: Arousal score (0 to 1)
            confidence: Confidence in emotion (0 to 1)
            context: Optional context dict
        
        Returns:
            Behavior command dictionary
        """
        try:
            # Get base behavior
            base_behavior = self.EMOTION_BEHAVIORS.get(
                emotion_label.lower(),
                self.EMOTION_BEHAVIORS['neutral']
            )
            
            behavior = self._deep_copy_dict(base_behavior)
            
            # Apply intensity modulation based on arousal
            intensity = self._get_intensity(arousal)
            behavior = self._apply_intensity(behavior, intensity)
            
            # Apply context modifiers
            if context:
                behavior = self._apply_context_modifiers(behavior, context)
            
            # Add metadata
            behavior['emotion'] = emotion_label
            behavior['valence'] = float(valence)
            behavior['arousal'] = float(arousal)
            behavior['confidence'] = float(confidence)
            
            # Ensure LED has intensity
            if 'led' not in behavior:
                behavior['led'] = {}
            if 'intensity' not in behavior['led']:
                behavior['led']['intensity'] = 0.8
            
            logger.debug(f'Mapped {emotion_label} to behavior: {behavior}')
            
            return behavior
        
        except Exception as e:
            logger.error(f'Behavior mapping error: {str(e)}')
            return self.EMOTION_BEHAVIORS['neutral']
    
    def _get_intensity(self, arousal: float) -> BehaviorIntensity:
        """Determine behavior intensity from arousal."""
        if arousal < 0.33:
            return BehaviorIntensity.SUBTLE
        elif arousal < 0.66:
            return BehaviorIntensity.NORMAL
        else:
            return BehaviorIntensity.STRONG
    
    def _apply_intensity(self, behavior: Dict, intensity: BehaviorIntensity) -> Dict:
        """Apply intensity modulation to behavior."""
        if intensity == BehaviorIntensity.SUBTLE:
            if 'led' in behavior:
                behavior['led']['intensity'] = 0.5
            # Keep gesture but make it gentle
            if 'gesture' in behavior:
                if 'bounce' in behavior['gesture']:
                    behavior['gesture'] = 'listen'
        
        elif intensity == BehaviorIntensity.STRONG:
            if 'led' in behavior:
                behavior['led']['intensity'] = 1.0
                if behavior['led']['pattern'] == 'steady':
                    behavior['led']['pattern'] = 'pulse'
        
        return behavior
    
    def _apply_context_modifiers(self, behavior: Dict, context: Dict) -> Dict:
        """Apply context-based modifiers to behavior."""
        try:
            # Adjust for group size
            if 'group_count' in context:
                count = context['group_count']
                modifier_key = 'high' if count > 2 else 'low'
                
                if modifier_key in self.CONTEXT_MODIFIERS.get('group_size', {}):
                    modifiers = self.CONTEXT_MODIFIERS['group_size'][modifier_key]
                    
                    if modifier_key == 'high' and 'led' in behavior:
                        behavior['led']['intensity'] = min(
                            1.0,
                            behavior['led'].get('intensity', 0.8) * 1.2
                        )
            
            # Adjust for proximity
            if 'proximity_m' in context:
                proximity = context['proximity_m']
                
                if proximity < 0.5:
                    if 'gesture' in behavior and 'bounce' in behavior['gesture']:
                        behavior['gesture'] = 'listen'
        
        except Exception as e:
            logger.debug(f'Context modifier error: {str(e)}')
        
        return behavior
    
    def _deep_copy_dict(self, d: Dict) -> Dict:
        """Deep copy dictionary."""
        return {k: dict(v) if isinstance(v, dict) else v for k, v in d.items()}
    
    def map_context_to_behavior(
        self,
        context: Dict
    ) -> Optional[Dict]:
        """
        Map contextual state to specific behavior.
        
        Args:
            context: Context dictionary
        
        Returns:
            Behavior dict or None
        """
        try:
            # If person detected approaching, greet them
            if context.get('group_count', 0) > 0 and context.get('proximity_m', 10) < 1.0:
                return {
                    'gesture': 'greet',
                    'led': {'color': 'cyan', 'pattern': 'steady', 'intensity': 0.8},
                    'tts': {'text': 'Hello! Welcome!', 'emotion': 'happy'},
                    'source': 'context_approach'
                }
            
            # If no one around, go idle
            if context.get('group_count', 0) == 0 and context.get('proximity_m', 10) > 2.0:
                return {
                    'gesture': 'idle',
                    'led': {'color': 'white', 'pattern': 'steady', 'intensity': 0.3},
                    'tts': {'text': '', 'emotion': 'neutral'},
                    'source': 'context_idle'
                }
            
            return None
        
        except Exception as e:
            logger.error(f'Context mapping error: {str(e)}')
            return None
    
    def create_idle_behavior(self) -> Dict:
        """Create idle/rest behavior."""
        return {
            'gesture': 'idle',
            'led': {'color': 'white', 'pattern': 'steady', 'intensity': 0.3},
            'tts': {'text': '', 'emotion': 'neutral'},
            'emotion': 'neutral',
            'valence': 0.0,
            'arousal': 0.1,
            'confidence': 1.0
        }
    
    def create_error_behavior(self, error_msg: str = '') -> Dict:
        """Create error/safe behavior."""
        return {
            'gesture': 'confused',
            'led': {'color': 'red', 'pattern': 'blink', 'intensity': 1.0},
            'tts': {'text': f'Error: {error_msg[:50]}', 'emotion': 'fearful'},
            'emotion': 'error',
            'confidence': 0.5
        }
