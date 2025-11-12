"""Tests for choreography engine."""

import pytest
import time
from sentio_demo.choreography_engine import ChoreographyEngine, PlaybackState


class TestChoreographyEngine:
    """Test suite for ChoreographyEngine."""
    
    @pytest.fixture
    def engine(self):
        """Create engine instance."""
        return ChoreographyEngine()
    
    @pytest.fixture
    def sample_sequence(self):
        """Create sample sequence."""
        return {
            'name': 'Test',
            'description': 'Test sequence',
            'total_duration_ms': 3000,
            'steps': [
                {
                    'index': 0,
                    'action': 'gesture',
                    'timestamp_ms': 0,
                    'data': {'gesture': 'greet'}
                },
                {
                    'index': 1,
                    'action': 'tts',
                    'timestamp_ms': 500,
                    'data': {'text': 'Hello'}
                },
                {
                    'index': 2,
                    'action': 'wait',
                    'timestamp_ms': 1500,
                    'data': {'duration_ms': 1000}
                }
            ]
        }
    
    def test_initial_state(self, engine):
        """Test initial engine state."""
        assert engine.state == PlaybackState.IDLE
        assert engine.current_sequence is None
    
    def test_register_callback(self, engine):
        """Test callback registration."""
        called = []
        
        def callback(data):
            called.append(data)
        
        engine.register_action_callback('test', callback)
        assert 'test' in engine.action_callbacks
    
    def test_play_sequence(self, engine, sample_sequence):
        """Test sequence playback."""
        engine.play(sample_sequence)
        assert engine.state == PlaybackState.PLAYING
        
        # Wait for completion
        time.sleep(1.0)
        
        # Stop to avoid blocking
        engine.stop()
    
    def test_pause_resume(self, engine, sample_sequence):
        """Test pause and resume."""
        engine.play(sample_sequence)
        
        time.sleep(0.5)
        engine.pause()
        assert engine.state == PlaybackState.PAUSED
        
        engine.resume()
        assert engine.state == PlaybackState.PLAYING
        
        engine.stop()
    
    def test_stop(self, engine, sample_sequence):
        """Test stop."""
        engine.play(sample_sequence)
        
        time.sleep(0.2)
        engine.stop()
        assert engine.state == PlaybackState.STOPPED
    
    def test_get_status(self, engine, sample_sequence):
        """Test status reporting."""
        engine.play(sample_sequence)
        
        status = engine.get_status()
        assert 'state' in status
        assert 'sequence_name' in status
        assert 'current_step' in status
        assert 'elapsed_ms' in status
        
        engine.stop()
    
    def test_action_callback_execution(self, engine, sample_sequence):
        """Test that callbacks are executed."""
        called_actions = []
        
        def gesture_callback(data):
            called_actions.append('gesture')
        
        def tts_callback(data):
            called_actions.append('tts')
        
        engine.register_action_callback('gesture', gesture_callback)
        engine.register_action_callback('tts', tts_callback)
        
        engine.play(sample_sequence)
        
        # Wait for execution
        time.sleep(2.0)
        
        engine.stop()
        
        # Check callbacks were called
        assert 'gesture' in called_actions
        assert 'tts' in called_actions
