#!/usr/bin/env python3
"""
Simulated Servo Backend for Testing

Listens on a virtual serial port and responds to servo commands.
Useful for testing without hardware.

Usage:
    python simulate_servo_backend.py &
    ros2 run sentio_motion servo_bridge_node --ros-args -p serial_port:/dev/pts/X
"""

import json
import sys
import logging
import time
import argparse
from typing import Dict, Optional

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class SimulatedServoBackend:
    """Simulates ESP32 servo bridge over serial port."""
    
    def __init__(self):
        """Initialize backend."""
        self.servo_positions: Dict[int, float] = {i: 90.0 for i in range(5)}
        self.command_count = 0
        self.error_count = 0
        self.emergency_stop = False
    
    def handle_command(self, command_str: str) -> str:
        """
        Handle a command and return response.
        
        Args:
            command_str: JSON command string
        
        Returns:
            JSON response string
        """
        try:
            command = json.loads(command_str)
        except json.JSONDecodeError as e:
            logger.error(f'JSON parse error: {str(e)}')
            self.error_count += 1
            return json.dumps({'status': 'error', 'msg': f'JSON parse error: {str(e)}'})
        
        cmd_type = command.get('type', '')
        
        if cmd_type == 'servo_move':
            return self._handle_servo_move(command)
        elif cmd_type == 'led_control':
            return self._handle_led_control(command)
        elif cmd_type == 'init':
            return self._handle_init(command)
        elif cmd_type == 'estop':
            self.emergency_stop = True
            return json.dumps({'status': 'ok', 'msg': 'Emergency stop activated'})
        elif cmd_type == 'clear_estop':
            self.emergency_stop = False
            return json.dumps({'status': 'ok', 'msg': 'Emergency stop cleared'})
        else:
            return json.dumps({'status': 'error', 'msg': f'Unknown command: {cmd_type}'})
    
    def _handle_servo_move(self, command: Dict) -> str:
        """Handle servo move command."""
        if self.emergency_stop:
            return json.dumps({'status': 'error', 'msg': 'Emergency stop active'})
        
        positions = command.get('positions', {})
        duration_ms = command.get('duration_ms', 500)
        
        response_positions = {}
        
        for servo_id_str, angle in positions.items():
            try:
                servo_id = int(servo_id_str)
                
                # Clamp angle
                angle = max(0.0, min(180.0, float(angle)))
                
                # Update position
                self.servo_positions[servo_id] = angle
                response_positions[servo_id] = angle
                
                logger.debug(f'Servo {servo_id} -> {angle}°')
            
            except (ValueError, TypeError) as e:
                logger.error(f'Invalid servo command: {str(e)}')
                self.error_count += 1
        
        self.command_count += 1
        
        return json.dumps({
            'status': 'ok',
            'positions': response_positions,
            'duration_ms': duration_ms,
            'cmd_count': self.command_count
        })
    
    def _handle_led_control(self, command: Dict) -> str:
        """Handle LED control command."""
        color = command.get('color', [255, 255, 255])
        pattern = command.get('pattern', 'solid')
        intensity = command.get('intensity', 0.8)
        
        logger.debug(f'LED: RGB{tuple(color)} pattern={pattern} intensity={intensity}')
        
        return json.dumps({'status': 'ok', 'msg': 'LED command executed'})
    
    def _handle_init(self, command: Dict) -> str:
        """Handle initialization command."""
        servo_count = command.get('servo_count', 5)
        
        logger.info(f'Initialization: {servo_count} servos')
        
        return json.dumps({
            'status': 'ok',
            'servo_count': servo_count,
            'version': '1.0.0'
        })
    
    def get_status(self) -> Dict:
        """Get backend status."""
        return {
            'servo_positions': self.servo_positions,
            'command_count': self.command_count,
            'error_count': self.error_count,
            'emergency_stop': self.emergency_stop
        }


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Simulated Servo Backend')
    parser.add_argument('--port', default='/dev/pts/0', help='Virtual serial port')
    args = parser.parse_args()
    
    backend = SimulatedServoBackend()
    
    logger.info('Simulated Servo Backend started')
    logger.info(f'Listening on {args.port}')
    logger.info('Commands are read from stdin in JSON format')
    
    try:
        while True:
            try:
                # Read command from stdin
                line = input('> ').strip()
                
                if not line:
                    continue
                
                if line == 'status':
                    print(json.dumps(backend.get_status(), indent=2))
                    continue
                
                if line == 'quit':
                    break
                
                # Handle command
                response = backend.handle_command(line)
                print(response)
            
            except KeyboardInterrupt:
                break
            except Exception as e:
                logger.error(f'Error: {str(e)}')
    
    finally:
        logger.info('Backend shutting down')


if __name__ == '__main__':
    main()
