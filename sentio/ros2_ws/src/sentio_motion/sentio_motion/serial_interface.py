"""Serial communication with ESP32 servo bridge."""

import serial
import json
import logging
import time
from typing import Optional, Tuple, Dict


logger = logging.getLogger(__name__)


class SerialInterface:
    """
    Low-level serial communication with ESP32 servo bridge.
    
    Protocol:
    - JSON frames terminated with newline
    - Request: {"type": "servo_move", "positions": {...}, ...}
    - Response: {"status": "ok", "positions": {...}} or {"status": "error", "msg": "..."}
    """
    
    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 115200,
        timeout: float = 1.0
    ):
        """
        Initialize serial interface.
        
        Args:
            port: Serial port device
            baudrate: Baud rate
            timeout: Read/write timeout
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.connected = False
    
    def connect(self) -> bool:
        """
        Establish serial connection.
        
        Returns:
            True if successful
        """
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=1.0
            )
            
            # Allow ESP32 to boot
            time.sleep(2.0)
            
            # Clear any buffered data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            self.connected = True
            logger.info(f'Serial connected: {self.port} @ {self.baudrate} baud')
            return True
        
        except serial.SerialException as e:
            logger.error(f'Serial connection failed: {str(e)}')
            self.connected = False
            return False
    
    def disconnect(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        logger.info('Serial disconnected')
    
    def send_command(self, command: Dict) -> Tuple[bool, Optional[Dict]]:
        """
        Send command to ESP32 and wait for response.
        
        Args:
            command: Command dictionary
        
        Returns:
            Tuple of (success, response)
        """
        if not self.connected or not self.ser:
            logger.error('Serial not connected')
            return False, None
        
        try:
            # Serialize command
            cmd_json = json.dumps(command) + '\n'
            
            # Send
            self.ser.write(cmd_json.encode('utf-8'))
            logger.debug(f'Sent: {cmd_json.strip()}')
            
            # Read response
            response_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            
            if response_line:
                response = json.loads(response_line)
                logger.debug(f'Received: {response}')
                
                status = response.get('status', 'unknown')
                return status == 'ok', response
            else:
                logger.warning('No response from ESP32')
                return False, None
        
        except json.JSONDecodeError as e:
            logger.error(f'JSON decode error: {str(e)}')
            return False, None
        except Exception as e:
            logger.error(f'Serial send error: {str(e)}')
            return False, None
    
    def is_connected(self) -> bool:
        """Check if connected."""
        return self.connected and self.ser and self.ser.is_open
