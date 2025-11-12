"""High-level motion controller."""

import logging
import json
from typing import Dict, Optional, List
from enum import Enum


logger = logging.getLogger(__name__)


class MotionState(Enum):
    """Motion controller states."""
    IDLE = 'idle'
    MOVING = 'moving'
    ERROR = 'error'
    EMERGENCY_STOP = 'emergency_stop'


class MotionController:
    """
    High-level motion controller managing servo transitions.
    
    Coordinates:
    - Gesture playback
    - Smooth transitions
    - Safety monitoring
    - Status reporting
    """
    
    def __init__(self):
        """Initialize motion controller."""
        self.state = MotionState.IDLE
        self.current_positions: Dict[int, float] = {}
        self.target_positions: Dict[int, float] = {}
        self.active_gesture: Optional[str] = None
        self.gesture_keyframe_index = 0
        self.command_count = 0
    
    def execute_gesture(
        self,
        gesture_name: str,
        gesture_def: Dict
    ) -> bool:
        """
        Execute a gesture.
        
        Args:
            gesture_name: Name of gesture
            gesture_def: Gesture definition dict
        
        Returns:
            True if successful
        """
        try:
            if 'keyframes' in gesture_def:
                # Multi-frame gesture
                self.active_gesture = gesture_name
                self.gesture_keyframe_index = 0
                logger.info(f'Starting keyframe gesture: {gesture_name}')
            elif 'positions' in gesture_def:
                # Single-frame gesture
                self.target_positions = dict(gesture_def['positions'])
                self.active_gesture = gesture_name
                logger.info(f'Executing gesture: {gesture_name}')
            else:
                logger.error(f'Invalid gesture definition for {gesture_name}')
                return False
            
            self.state = MotionState.MOVING
            self.command_count += 1
            return True
        
        except Exception as e:
            logger.error(f'Gesture execution error: {str(e)}')
            self.state = MotionState.ERROR
            return False
    
    def get_next_keyframe(self, gesture_def: Dict) -> Optional[Dict]:
        """
        Get next keyframe in gesture sequence.
        
        Args:
            gesture_def: Gesture definition
        
        Returns:
            Keyframe dict or None if sequence complete
        """
        if 'keyframes' not in gesture_def:
            return None
        
        keyframes = gesture_def['keyframes']
        
        if self.gesture_keyframe_index >= len(keyframes):
            self.gesture_keyframe_index = 0
            self.active_gesture = None
            return None
        
        keyframe = keyframes[self.gesture_keyframe_index]
        self.gesture_keyframe_index += 1
        
        return keyframe
    
    def update_position(self, servo_id: int, angle: float):
        """Update servo position."""
        self.current_positions[servo_id] = angle
    
    def get_motion_status(self) -> Dict:
        """Get current motion status."""
        return {
            'state': self.state.value,
            'active_gesture': self.active_gesture,
            'current_positions': self.current_positions,
            'target_positions': self.target_positions,
            'commands_sent': self.command_count
        }
