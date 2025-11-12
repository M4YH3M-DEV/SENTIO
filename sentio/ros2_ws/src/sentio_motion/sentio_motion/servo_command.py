"""Servo command structures and serialization."""

import json
import logging
from typing import Dict, List, Optional


logger = logging.getLogger(__name__)


class ServoCommand:
    """
    Represents a servo motion command.
    
    Format (JSON):
    {
        "type": "servo_move",
        "positions": {"servo_id": angle_deg, ...},
        "duration_ms": 500,
        "velocity_limit": 60.0,
        "smoothing": true
    }
    """
    
    def __init__(self):
        """Initialize servo command."""
        self.positions: Dict[int, float] = {}
        self.duration_ms = 500
        self.velocity_limit = 60.0  # degrees per second
        self.smoothing_enabled = True
    
    def add_servo(self, servo_id: int, angle_deg: float):
        """Add servo position target."""
        self.positions[servo_id] = float(angle_deg)
    
    def to_json(self) -> str:
        """Serialize to JSON string."""
        data = {
            'type': 'servo_move',
            'positions': self.positions,
            'duration_ms': int(self.duration_ms),
            'velocity_limit': float(self.velocity_limit),
            'smoothing': self.smoothing_enabled
        }
        return json.dumps(data)
    
    @staticmethod
    def from_json(json_str: str) -> Optional['ServoCommand']:
        """Deserialize from JSON string."""
        try:
            data = json.loads(json_str)
            cmd = ServoCommand()
            cmd.positions = data.get('positions', {})
            cmd.duration_ms = data.get('duration_ms', 500)
            cmd.velocity_limit = data.get('velocity_limit', 60.0)
            cmd.smoothing_enabled = data.get('smoothing', True)
            return cmd
        except Exception as e:
            logger.error(f'Failed to deserialize servo command: {str(e)}')
            return None
    
    def __str__(self) -> str:
        return f'ServoCommand({len(self.positions)} servos, {self.duration_ms}ms)'


class LEDCommand:
    """
    Represents an LED control command.
    
    Format (JSON):
    {
        "type": "led_control",
        "color": [R, G, B],
        "pattern": "solid|pulse|blink",
        "intensity": 0.8
    }
    """
    
    def __init__(self, color: tuple = (255, 255, 255), pattern: str = 'solid', intensity: float = 0.8):
        """Initialize LED command."""
        self.color = color
        self.pattern = pattern
        self.intensity = float(max(0.0, min(1.0, intensity)))
    
    def to_json(self) -> str:
        """Serialize to JSON string."""
        data = {
            'type': 'led_control',
            'color': list(self.color),
            'pattern': self.pattern,
            'intensity': self.intensity
        }
        return json.dumps(data)
    
    def to_color_code(self) -> int:
        """Convert color to 24-bit RGB integer."""
        r, g, b = self.color
        return (int(r) << 16) | (int(g) << 8) | int(b)


class CombinedCommand:
    """Combines servo and LED commands."""
    
    def __init__(self):
        """Initialize combined command."""
        self.servo_cmd: Optional[ServoCommand] = None
        self.led_cmd: Optional[LEDCommand] = None
    
    def to_json(self) -> str:
        """Serialize to JSON string."""
        data = {
            'servo': json.loads(self.servo_cmd.to_json()) if self.servo_cmd else None,
            'led': json.loads(self.led_cmd.to_json()) if self.led_cmd else None
        }
        return json.dumps(data)
