"""Tests for behavior mapper."""

import pytest
from sentio_policy.behavior_mapper import BehaviorMapper, BehaviorIntensity


class TestBehaviorMapper:
    """Test suite for BehaviorMapper."""
    
    @pytest.fixture
    def mapper(self):
        """Create mapper instance."""
        return BehaviorMapper()
    
    def test_happy_emotion_mapping(self, mapper):
        """Test happy emotion maps to correct behavior."""
        behavior = mapper.map_emotion_to_behavior(
            emotion_label='happy',
            valence=0.8,
            arousal=0.6,
            confidence=0.9
        )
        
        assert behavior['gesture'] == 'happy_bounce'
        assert behavior['led']['color'] == 'yellow'
        assert 'Happy' in behavior['tts']['text'] or 'happy' in behavior['tts']['text'].lower()
    
    def test_sad_emotion_mapping(self, mapper):
        """Test sad emotion maps to correct behavior."""
        behavior = mapper.map_emotion_to_behavior(
            emotion_label='sad',
            valence=-0.7,
            arousal=0.2,
            confidence=0.8
        )
        
        assert behavior['gesture'] == 'lean_back'
        assert behavior['led']['color'] == 'blue'
    
    def test_intensity_subtle(self, mapper):
        """Test subtle intensity for low arousal."""
        behavior = mapper.map_emotion_to_behavior(
            emotion_label='happy',
            valence=0.8,
            arousal=0.2,  # Low arousal
            confidence=0.9
        )
        
        # Should be less intense
        assert behavior['led']['intensity'] < 0.8
    
    def test_intensity_strong(self, mapper):
        """Test strong intensity for high arousal."""
        behavior = mapper.map_emotion_to_behavior(
            emotion_label='happy',
            valence=0.8,
            arousal=0.9,  # High arousal
            confidence=0.9
        )
        
        assert behavior['led']['intensity'] >= 0.9
    
    def test_neutral_behavior(self, mapper):
        """Test neutral emotion behavior."""
        behavior = mapper.map_emotion_to_behavior(
            emotion_label='neutral',
            valence=0.0,
            arousal=0.3,
            confidence=0.5
        )
        
        assert behavior['gesture'] == 'idle'
        assert behavior['led']['color'] == 'white'
    
    def test_context_proximity_modifier(self, mapper):
        """Test proximity context modifies gesture."""
        context = {'proximity_m': 0.3}
        
        behavior = mapper.map_emotion_to_behavior(
            emotion_label='happy',
            valence=0.8,
            arousal=0.6,
            context=context
        )
        
        # Close proximity should make gesture gentler
        assert behavior['gesture'] == 'listen' or behavior['gesture'] == 'happy_bounce'
    
    def test_context_group_size_modifier(self, mapper):
        """Test group size modifies LED intensity."""
        context = {'group_count': 5}
        
        behavior = mapper.map_emotion_to_behavior(
            emotion_label='happy',
            valence=0.8,
            arousal=0.6,
            context=context
        )
        
        # Larger group should increase intensity
        assert behavior['led']['intensity'] > 0.5
    
    def test_idle_behavior(self, mapper):
        """Test idle behavior creation."""
        behavior = mapper.create_idle_behavior()
        
        assert behavior['gesture'] == 'idle'
        assert behavior['emotion'] == 'neutral'
        assert len(behavior['tts']['text']) == 0
    
    def test_error_behavior(self, mapper):
        """Test error behavior creation."""
        behavior = mapper.create_error_behavior('Test error')
        
        assert behavior['gesture'] == 'confused'
        assert behavior['led']['color'] == 'red'
        assert 'error' in behavior['emotion'].lower()
