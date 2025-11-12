"""
SENTIO Drivers Package
======================

Low-level hardware interface drivers for SENTIO sensors:
- TFMini-S LiDAR (distance/proximity)
- MaxSonar ultrasonic range sensor
- MPU-9250 IMU (orientation, acceleration)
- USB camera (vision input)
- ReSpeaker microphone array (audio input)

All drivers publish to standardized ROS 2 sensor topics.

Author: DevSora Deep-Tech Research
License: Proprietary
"""

__version__ = '0.1.0'
__author__ = 'DevSora Team'
__license__ = 'Proprietary'

from .device_manager import DeviceManager

__all__ = ['DeviceManager']
