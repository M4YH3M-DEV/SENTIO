"""Kinematics engine for gesture and pose mapping."""

import logging
import yaml
from typing import Dict, List, Optional
from pathlib import Path


logger = logging.getLogger(__name__)


class KinematicsEngine:
    """
    Manages servo kinematics and gesture definitions.
    
    Maps high-level gestures to servo target positions.
    """
    
    def __init__(self, config_file: str = ''):
        """
        Initialize kinematics engine.
        
        Args:
            config_file: Path to kinematics YAML config
        """
        self.servo_map: Dict[str, int] = {}
        self.neutral_angles: Dict[int, float] = {}
        self.min_angles: Dict[int, float] = {}
        self.max_angles: Dict[int, float] = {}
        self.gestures: Dict[str, Dict] = {}
        
        if config_file and Path(config_file).exists():
            self.load_config(config_file)
        else:
            self._load_default_config()
    
    def _load_default_config(self):
        """Load default kinematics configuration."""
        self.servo_map = {
            'neck_pan': 0,
            'neck_tilt': 1,
            'torso_rotate': 2,
            'left_shoulder': 3,
            'right_shoulder': 4,
        }
        
        self.neutral_angles = {
            0: 90.0,  # Neck pan
            1: 90.0,  # Neck tilt
            2: 90.0,  # Torso
            3: 90.0,  # Shoulder
            4: 90.0,  # Shoulder
        }
        
        self.min_angles = {i: 0.0 for i in range(5)}
        self.max_angles = {i: 180.0 for i in range(5)}
        
        self.gestures = {
            'idle': {
                'description': 'Neutral standing position',
                'positions': {0: 90, 1: 90, 2: 90, 3: 90, 4: 90},
                'duration_ms': 500
            },
            'greet': {
                'description': 'Wave hand gesture',
                'positions': {4: 45},  # Raise right shoulder
                'duration_ms': 1000
            },
            'nod': {
                'description': 'Nod head yes',
                'keyframes': [
                    {'positions': {1: 100}, 'duration_ms': 300},
                    {'positions': {1: 80}, 'duration_ms': 300},
                    {'positions': {1: 90}, 'duration_ms': 300}
                ]
            },
            'shake_head': {
                'description': 'Shake head no',
                'keyframes': [
                    {'positions': {0: 110}, 'duration_ms': 250},
                    {'positions': {0: 70}, 'duration_ms': 250},
                    {'positions': {0: 90}, 'duration_ms': 250}
                ]
            },
            'look_left': {
                'description': 'Look to the left',
                'positions': {0: 135},
                'duration_ms': 500
            },
            'look_right': {
                'description': 'Look to the right',
                'positions': {0: 45},
                'duration_ms': 500
            },
            'happy_bounce': {
                'description': 'Happy bouncing motion',
                'keyframes': [
                    {'positions': {3: 70, 4: 70}, 'duration_ms': 200},
                    {'positions': {3: 90, 4: 90}, 'duration_ms': 200},
                    {'positions': {3: 70, 4: 70}, 'duration_ms': 200},
                    {'positions': {3: 90, 4: 90}, 'duration_ms': 200}
                ]
            },
        }
        
        logger.info('Loaded default kinematics configuration')
    
    def load_config(self, config_file: str):
        """
        Load kinematics configuration from YAML file.
        
        Args:
            config_file: Path to YAML file
        """
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            self.servo_map = config.get('servo_map', {})
            self.neutral_angles = config.get('neutral_angles', {})
            self.min_angles = config.get('min_angles', {})
            self.max_angles = config.get('max_angles', {})
            self.gestures = config.get('gestures', {})
            
            logger.info(f'Loaded kinematics from {config_file}')
        
        except Exception as e:
            logger.error(f'Failed to load kinematics config: {str(e)}')
            self._load_default_config()
    
    def get_gesture(self, gesture_name: str) -> Optional[Dict]:
        """
        Get gesture definition.
        
        Args:
            gesture_name: Name of gesture
        
        Returns:
            Gesture dictionary or None
        """
        return self.gestures.get(gesture_name.lower())
    
    def get_neutral_pose(self) -> Dict[int, float]:
        """Get neutral/idle pose."""
        return dict(self.neutral_angles)
    
    def clamp_angle(self, servo_id: int, angle: float) -> float:
        """
        Clamp angle to servo limits.
        
        Args:
            servo_id: Servo identifier
            angle: Desired angle
        
        Returns:
            Clamped angle
        """
        min_angle = self.min_angles.get(servo_id, 0.0)
        max_angle = self.max_angles.get(servo_id, 180.0)
        return max(min_angle, min(max_angle, angle))
    
    def interpolate_keyframes(
        self,
        keyframes: List[Dict],
        total_duration_ms: Optional[int] = None
    ) -> List[Dict]:
        """
        Interpolate motion between keyframes.
        
        Args:
            keyframes: List of keyframe dicts
            total_duration_ms: Optional total duration override
        
        Returns:
            List of interpolated frames
        """
        if not keyframes or len(keyframes) < 2:
            return keyframes
        
        interpolated = []
        
        for i in range(len(keyframes) - 1):
            frame1 = keyframes[i]
            frame2 = keyframes[i + 1]
            
            positions1 = frame1.get('positions', {})
            positions2 = frame2.get('positions', {})
            
            duration = frame2.get('duration_ms', 500)
            steps = max(2, duration // 50)  # 50ms resolution
            
            for step in range(steps + 1):
                t = step / steps
                interpolated_positions = {}
                
                # Interpolate all servo positions
                all_servos = set(positions1.keys()) | set(positions2.keys())
                for servo_id in all_servos:
                    p1 = positions1.get(servo_id, self.neutral_angles.get(servo_id, 90.0))
                    p2 = positions2.get(servo_id, self.neutral_angles.get(servo_id, 90.0))
                    interpolated_positions[servo_id] = p1 + t * (p2 - p1)
                
                interpolated.append({
                    'positions': interpolated_positions,
                    'duration_ms': 50
                })
        
        return interpolated
