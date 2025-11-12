"""
Validators for Behavior Commands

Validates behavior payloads against schema.
"""

import json
import logging
from typing import Dict, Tuple, Optional
from pathlib import Path


logger = logging.getLogger(__name__)


class BehaviorValidator:
    """
    Validates behavior command structures against schema.
    """
    
    # Inline schema for behavior commands
    BEHAVIOR_SCHEMA = {
        'type': 'object',
        'required': ['gesture', 'led', 'tts'],
        'properties': {
            'gesture': {
                'type': 'string',
                'enum': [
                    'idle', 'greet', 'wave', 'nod', 'shake_head',
                    'lean_forward', 'lean_back', 'listen', 'think',
                    'confused', 'surprised', 'happy_bounce'
                ]
            },
            'led': {
                'type': 'object',
                'required': ['color', 'pattern'],
                'properties': {
                    'color': {
                        'type': 'string',
                        'enum': [
                            'off', 'white', 'red', 'green', 'blue',
                            'yellow', 'cyan', 'magenta', 'orange'
                        ]
                    },
                    'pattern': {
                        'type': 'string',
                        'enum': ['steady', 'pulse', 'blink', 'rainbow', 'chase']
                    },
                    'intensity': {
                        'type': 'number',
                        'minimum': 0.0,
                        'maximum': 1.0
                    }
                }
            },
            'tts': {
                'type': 'object',
                'required': ['text'],
                'properties': {
                    'text': {
                        'type': 'string',
                        'maxLength': 500
                    },
                    'voice': {
                        'type': 'string'
                    },
                    'emotion': {
                        'type': 'string'
                    }
                }
            },
            'duration_s': {
                'type': 'number',
                'minimum': 0.0,
                'maximum': 300.0
            }
        }
    }
    
    def __init__(self, schema_path: Optional[str] = None):
        """
        Initialize validator.
        
        Args:
            schema_path: Optional path to JSON schema file
        """
        self.schema = self._load_schema(schema_path)
    
    def _load_schema(self, schema_path: Optional[str]) -> Dict:
        """Load schema from file or use default."""
        if schema_path and Path(schema_path).exists():
            try:
                with open(schema_path, 'r') as f:
                    return json.load(f)
            except Exception as e:
                logger.warning(f'Failed to load schema from {schema_path}: {str(e)}')
        
        return self.BEHAVIOR_SCHEMA
    
    def validate(self, behavior: Dict) -> Tuple[bool, Optional[str]]:
        """
        Validate behavior command.
        
        Args:
            behavior: Behavior dictionary
        
        Returns:
            Tuple of (is_valid, error_message)
        """
        # Check required fields
        required = self.schema.get('required', [])
        for field in required:
            if field not in behavior:
                return False, f'Missing required field: {field}'
        
        # Check gesture
        if 'gesture' in behavior:
            valid_gestures = self.schema['properties']['gesture']['enum']
            if behavior['gesture'] not in valid_gestures:
                return False, f'Invalid gesture: {behavior["gesture"]}'
        
        # Check LED
        if 'led' in behavior:
            led_valid, led_err = self._validate_led(behavior['led'])
            if not led_valid:
                return False, led_err
        
        # Check TTS
        if 'tts' in behavior:
            tts_valid, tts_err = self._validate_tts(behavior['tts'])
            if not tts_valid:
                return False, tts_err
        
        # Check duration
        if 'duration_s' in behavior:
            if not isinstance(behavior['duration_s'], (int, float)):
                return False, 'duration_s must be numeric'
            if behavior['duration_s'] < 0 or behavior['duration_s'] > 300:
                return False, 'duration_s out of range [0-300]'
        
        return True, None
    
    def _validate_led(self, led: Dict) -> Tuple[bool, Optional[str]]:
        """Validate LED command."""
        if not isinstance(led, dict):
            return False, 'led must be an object'
        
        # Check required LED fields
        if 'color' not in led or 'pattern' not in led:
            return False, 'led missing required fields (color, pattern)'
        
        valid_colors = self.schema['properties']['led']['properties']['color']['enum']
        if led['color'] not in valid_colors:
            return False, f'Invalid LED color: {led["color"]}'
        
        valid_patterns = self.schema['properties']['led']['properties']['pattern']['enum']
        if led['pattern'] not in valid_patterns:
            return False, f'Invalid LED pattern: {led["pattern"]}'
        
        if 'intensity' in led:
            if not isinstance(led['intensity'], (int, float)):
                return False, 'LED intensity must be numeric'
            if led['intensity'] < 0 or led['intensity'] > 1:
                return False, 'LED intensity out of range [0-1]'
        
        return True, None
    
    def _validate_tts(self, tts: Dict) -> Tuple[bool, Optional[str]]:
        """Validate TTS command."""
        if not isinstance(tts, dict):
            return False, 'tts must be an object'
        
        if 'text' not in tts:
            return False, 'tts missing required field: text'
        
        if not isinstance(tts['text'], str):
            return False, 'tts text must be string'
        
        if len(tts['text']) > 500:
            return False, 'tts text exceeds 500 character limit'
        
        if len(tts['text']) == 0:
            return False, 'tts text cannot be empty'
        
        return True, None
