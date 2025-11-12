"""Tests for behavior validators."""

import pytest
from sentio_policy.validators import BehaviorValidator


class TestBehaviorValidator:
    """Test suite for BehaviorValidator."""
    
    @pytest.fixture
    def validator(self):
        """Create validator instance."""
        return BehaviorValidator()
    
    @pytest.fixture
    def valid_behavior(self):
        """Valid behavior example."""
        return {
            'gesture': 'wave',
            'led': {
                'color': 'yellow',
                'pattern': 'pulse',
                'intensity': 0.8
            },
            'tts': {
                'text': 'Hello!'
            }
        }
    
    def test_valid_behavior(self, validator, valid_behavior):
        """Test validation of valid behavior."""
        is_valid, error = validator.validate(valid_behavior)
        assert is_valid is True
        assert error is None
    
    def test_missing_required_field(self, validator):
        """Test missing required field."""
        behavior = {
            'gesture': 'wave',
            'led': {'color': 'yellow', 'pattern': 'pulse'}
            # Missing 'tts'
        }
        
        is_valid, error = validator.validate(behavior)
        assert is_valid is False
        assert 'tts' in error.lower()
    
    def test_invalid_gesture(self, validator, valid_behavior):
        """Test invalid gesture name."""
        valid_behavior['gesture'] = 'invalid_gesture'
        
        is_valid, error = validator.validate(valid_behavior)
        assert is_valid is False
        assert 'gesture' in error.lower()
    
    def test_invalid_led_color(self, validator, valid_behavior):
        """Test invalid LED color."""
        valid_behavior['led']['color'] = 'purple'
        
        is_valid, error = validator.validate(valid_behavior)
        assert is_valid is False
        assert 'color' in error.lower()
    
    def test_invalid_led_pattern(self, validator, valid_behavior):
        """Test invalid LED pattern."""
        valid_behavior['led']['pattern'] = 'strobe'
        
        is_valid, error = validator.validate(valid_behavior)
        assert is_valid is False
        assert 'pattern' in error.lower()
    
    def test_intensity_out_of_range(self, validator, valid_behavior):
        """Test LED intensity out of range."""
        valid_behavior['led']['intensity'] = 1.5
        
        is_valid, error = validator.validate(valid_behavior)
        assert is_valid is False
        assert 'intensity' in error.lower()
    
    def test_empty_tts_text(self, validator, valid_behavior):
        """Test empty TTS text."""
        valid_behavior['tts']['text'] = ''
        
        is_valid, error = validator.validate(valid_behavior)
        assert is_valid is False
        assert 'empty' in error.lower()
    
    def test_long_tts_text(self, validator, valid_behavior):
        """Test TTS text exceeding limit."""
        valid_behavior['tts']['text'] = 'x' * 600
        
        is_valid, error = validator.validate(valid_behavior)
        assert is_valid is False
        assert '500' in error
    
    def test_duration_valid(self, validator, valid_behavior):
        """Test valid duration."""
        valid_behavior['duration_s'] = 5.0
        
        is_valid, error = validator.validate(valid_behavior)
        assert is_valid is True
    
    def test_duration_out_of_range(self, validator, valid_behavior):
        """Test duration out of range."""
        valid_behavior['duration_s'] = 400.0
        
        is_valid, error = validator.validate(valid_behavior)
        assert is_valid is False
        assert 'range' in error.lower()
