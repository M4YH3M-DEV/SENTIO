"""
SENTIO Motion Package
====================

Motion control and actuation layer for SENTIO robot.

Features:
- Servo position control via serial/I2C
- Kinematics and gesture mapping
- Safety monitoring (IMU, velocity limits, torque)
- Smooth motion trajectories
- Comprehensive error handling

Author: DevSora Deep-Tech Research
License: Proprietary
"""

__version__ = '0.1.0'
__author__ = 'DevSora Team'
__license__ = 'Proprietary'

from .kinematics_engine import KinematicsEngine
from .motion_controller import MotionController
from .safety_monitor import SafetyMonitor

__all__ = ['KinematicsEngine', 'MotionController', 'SafetyMonitor']
