"""Tests for safety monitoring."""

import pytest
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from sentio_motion.safety_monitor import SafetyMonitor


class TestSafetyMonitor:
    """Test suite for SafetyMonitor."""
    
    @pytest.fixture
    def monitor(self):
        """Create safety monitor instance."""
        return SafetyMonitor(
            max_velocity_dps=60.0,
            fall_threshold_g=1.5,
            torque_limit_nm=5.0
        )
    
    def test_velocity_clamping_within_limit(self, monitor):
        """Test velocity clamping within limits."""
        target, duration = monitor.clamp_velocity(
            current_pos=0.0,
            target_pos=30.0,
            duration_ms=1000
        )
        
        assert target == 30.0
        assert duration == 1000
    
    def test_velocity_clamping_exceeds_limit(self, monitor):
        """Test velocity clamping when exceeding limit."""
        # 90 degrees in 100ms = 900 dps, exceeds 60 dps limit
        target, duration = monitor.clamp_velocity(
            current_pos=0.0,
            target_pos=90.0,
            duration_ms=100
        )
        
        assert target == 90.0
        assert duration > 100  # Duration should be extended
    
    def test_fall_detection_no_data(self, monitor):
        """Test fall detection with no IMU data."""
        is_falling, reason = monitor.check_fall()
        assert is_falling is False
    
    def test_fall_detection_normal(self, monitor):
        """Test fall detection with normal acceleration."""
        imu = Imu()
        imu.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
        
        monitor.update_imu(imu)
        is_falling, reason = monitor.check_fall()
        
        assert is_falling is False
    
    def test_fall_detection_high_acceleration(self, monitor):
        """Test fall detection with high acceleration."""
        imu = Imu()
        # Create acceleration larger than fall threshold
        accel_magnitude = 2.0 * 9.81  # 2G
        imu.linear_acceleration = Vector3(x=accel_magnitude, y=0.0, z=0.0)
        
        monitor.update_imu(imu)
        is_falling, reason = monitor.check_fall()
        
        assert is_falling is True
    
    def test_emergency_stop_activation(self, monitor):
        """Test emergency stop activation."""
        assert monitor.check_emergency_stop() is False
        
        monitor.activate_emergency_stop('Test reason')
        assert monitor.check_emergency_stop() is True
        
        monitor.deactivate_emergency_stop()
        assert monitor.check_emergency_stop() is False
    
    def test_safety_status(self, monitor):
        """Test getting safety status."""
        status = monitor.get_safety_status()
        
        assert 'emergency_stop_active' in status
        assert 'is_falling' in status
        assert 'violations_count' in status
