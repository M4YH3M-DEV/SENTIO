"""Safety monitoring for motion control."""

import logging
import math
from typing import Optional, Dict, Tuple
from sensor_msgs.msg import Imu


logger = logging.getLogger(__name__)


class SafetyMonitor:
    """
    Monitors safety conditions during motion.
    
    Checks:
    - IMU for abnormal accelerations (fall detection)
    - Velocity limits
    - Torque limits
    - Emergency stop conditions
    """
    
    def __init__(
        self,
        max_velocity_dps: float = 60.0,
        max_acceleration_mss: float = 2.0,
        fall_threshold_g: float = 1.5,
        torque_limit_nm: float = 5.0
    ):
        """
        Initialize safety monitor.
        
        Args:
            max_velocity_dps: Maximum velocity degrees per second
            max_acceleration_mss: Maximum acceleration m/s²
            fall_threshold_g: Fall detection threshold in Gs
            torque_limit_nm: Maximum servo torque in Nm
        """
        self.max_velocity_dps = max_velocity_dps
        self.max_acceleration_mss = max_acceleration_mss
        self.fall_threshold_g = fall_threshold_g
        self.torque_limit_nm = torque_limit_nm
        
        self.last_imu_data: Optional[Imu] = None
        self.emergency_stop_active = False
        self.safety_violations: list = []
    
    def update_imu(self, imu_data: Imu):
        """Update with latest IMU data."""
        self.last_imu_data = imu_data
    
    def check_fall(self) -> Tuple[bool, Optional[str]]:
        """
        Check for fall condition.
        
        Returns:
            Tuple of (is_falling, reason)
        """
        if not self.last_imu_data:
            return False, None
        
        accel = self.last_imu_data.linear_acceleration
        accel_magnitude = math.sqrt(
            accel.x**2 + accel.y**2 + accel.z**2
        )
        
        # Convert to Gs (divide by ~9.81)
        accel_g = accel_magnitude / 9.81
        
        if accel_g > self.fall_threshold_g:
            return True, f'High acceleration detected: {accel_g:.2f}G'
        
        return False, None
    
    def clamp_velocity(
        self,
        current_pos: float,
        target_pos: float,
        duration_ms: int
    ) -> Tuple[float, int]:
        """
        Clamp motion to velocity limit.
        
        Args:
            current_pos: Current servo position
            target_pos: Target servo position
            duration_ms: Requested duration
        
        Returns:
            Tuple of (adjusted_target, adjusted_duration)
        """
        angle_delta = abs(target_pos - current_pos)
        
        # Calculate minimum duration to stay within velocity limit
        min_duration_ms = (angle_delta / self.max_velocity_dps) * 1000.0
        
        if duration_ms < min_duration_ms:
            logger.warning(
                f'Clamping velocity: {angle_delta}° in {duration_ms}ms '
                f'exceeds limit, extending to {min_duration_ms:.0f}ms'
            )
            return target_pos, int(min_duration_ms)
        
        return target_pos, duration_ms
    
    def check_emergency_stop(self) -> bool:
        """Check if emergency stop is active."""
        return self.emergency_stop_active
    
    def activate_emergency_stop(self, reason: str = ''):
        """Activate emergency stop."""
        self.emergency_stop_active = True
        logger.error(f'EMERGENCY STOP activated: {reason}')
        self.safety_violations.append(reason)
    
    def deactivate_emergency_stop(self):
        """Deactivate emergency stop."""
        self.emergency_stop_active = False
        logger.info('Emergency stop deactivated')
    
    def get_safety_status(self) -> Dict:
        """Get current safety status."""
        is_falling, fall_reason = self.check_fall()
        
        return {
            'emergency_stop_active': self.emergency_stop_active,
            'is_falling': is_falling,
            'fall_reason': fall_reason,
            'violations_count': len(self.safety_violations),
            'max_velocity_dps': self.max_velocity_dps,
            'fall_threshold_g': self.fall_threshold_g
        }
