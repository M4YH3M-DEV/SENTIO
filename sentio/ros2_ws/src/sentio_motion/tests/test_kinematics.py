"""Tests for kinematics engine."""

import pytest
from sentio_motion.kinematics_engine import KinematicsEngine


class TestKinematicsEngine:
    """Test suite for KinematicsEngine."""
    
    @pytest.fixture
    def engine(self):
        """Create kinematics engine instance."""
        return KinematicsEngine()
    
    def test_default_config_loaded(self, engine):
        """Test default configuration is loaded."""
        assert len(engine.servo_map) > 0
        assert len(engine.neutral_angles) > 0
        assert len(engine.gestures) > 0
    
    def test_get_gesture(self, engine):
        """Test getting gesture definition."""
        gesture = engine.get_gesture('idle')
        assert gesture is not None
        assert 'positions' in gesture or 'keyframes' in gesture
    
    def test_unknown_gesture(self, engine):
        """Test getting unknown gesture returns None."""
        gesture = engine.get_gesture('unknown_gesture_xyz')
        assert gesture is None
    
    def test_get_neutral_pose(self, engine):
        """Test getting neutral pose."""
        neutral = engine.get_neutral_pose()
        assert len(neutral) > 0
        assert all(0 <= angle <= 180 for angle in neutral.values())
    
    def test_clamp_angle_within_range(self, engine):
        """Test clamping angle within range."""
        clamped = engine.clamp_angle(0, 90)
        assert clamped == 90
    
    def test_clamp_angle_below_minimum(self, engine):
        """Test clamping angle below minimum."""
        clamped = engine.clamp_angle(0, -10)
        assert clamped == 0.0  # Min angle
    
    def test_clamp_angle_above_maximum(self, engine):
        """Test clamping angle above maximum."""
        clamped = engine.clamp_angle(0, 200)
        assert clamped == 180.0  # Max angle
    
    def test_interpolate_keyframes(self, engine):
        """Test keyframe interpolation."""
        keyframes = [
            {'positions': {0: 90}, 'duration_ms': 500},
            {'positions': {0: 135}, 'duration_ms': 500}
        ]
        
        interpolated = engine.interpolate_keyframes(keyframes)
        
        # Should have multiple frames
        assert len(interpolated) > 2
        
        # Values should transition from 90 to 135
        first_angle = interpolated[0]['positions'][0]
        last_angle = interpolated[-1]['positions'][0]
        
        assert first_angle < last_angle
