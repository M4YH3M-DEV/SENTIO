"""
Utility functions for hardware driver operations.

Provides serial communication, error handling, and data validation utilities.
"""

import serial
import logging
from typing import Optional, Tuple
from datetime import datetime


logger = logging.getLogger(__name__)


def setup_serial_connection(
    port: str,
    baudrate: int = 115200,
    timeout: float = 1.0,
    max_retries: int = 3
) -> Optional[serial.Serial]:
    """
    Establish a serial connection to a device.
    
    Args:
        port: Device port (e.g., '/dev/ttyUSB0')
        baudrate: Communication speed (default: 115200)
        timeout: Read timeout in seconds (default: 1.0)
        max_retries: Number of connection attempts (default: 3)
    
    Returns:
        Serial object if successful, None if failed after retries
    """
    for attempt in range(max_retries):
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=1.0
            )
            logger.info(f'Serial connection established: {port} @ {baudrate} baud')
            return ser
        except serial.SerialException as e:
            logger.warning(
                f'Serial connection attempt {attempt + 1}/{max_retries} failed: {str(e)}'
            )
            if attempt == max_retries - 1:
                logger.error(f'Failed to connect to {port} after {max_retries} attempts')
                return None
    
    return None


def safe_serial_read(
    ser: serial.Serial,
    num_bytes: int = 1,
    timeout_ms: int = 100
) -> Optional[bytes]:
    """
    Safely read data from serial port with error handling.
    
    Args:
        ser: Serial port object
        num_bytes: Number of bytes to read
        timeout_ms: Timeout in milliseconds
    
    Returns:
        Bytes read, or None if error
    """
    try:
        if ser and ser.is_open:
            ser.timeout = timeout_ms / 1000.0
            data = ser.read(num_bytes)
            return data if len(data) == num_bytes else None
    except Exception as e:
        logger.debug(f'Serial read error: {str(e)}')
    
    return None


def safe_serial_write(
    ser: serial.Serial,
    data: bytes
) -> bool:
    """
    Safely write data to serial port with error handling.
    
    Args:
        ser: Serial port object
        data: Bytes to write
    
    Returns:
        True if successful, False otherwise
    """
    try:
        if ser and ser.is_open:
            ser.write(data)
            return True
    except Exception as e:
        logger.debug(f'Serial write error: {str(e)}')
    
    return False


def parse_tfmini_frame(data: bytes) -> Optional[Tuple[float, int, int]]:
    """
    Parse TFMini-S distance measurement frame.
    
    Protocol: 7-byte frame
    [0x59, 0x59, dist_lo, dist_hi, strength_lo, strength_hi, mode]
    
    Args:
        data: Raw 7-byte frame
    
    Returns:
        Tuple of (distance_cm, signal_strength, mode) or None if invalid
    """
    if not data or len(data) < 7:
        return None
    
    if data[0] != 0x59 or data[1] != 0x59:
        return None  # Invalid header
    
    try:
        dist = (data[3] << 8) | data[2]  # Distance in cm
        strength = (data[5] << 8) | data[4]  # Signal strength
        mode = data[6]
        
        return (float(dist) / 100.0, strength, mode)  # Return distance in meters
    except (IndexError, ValueError):
        return None


def parse_maxsonar_ascii(data: bytes) -> Optional[float]:
    """
    Parse MaxSonar ASCII output format.
    
    Format: 'Rxxx\r' where xxx is distance in cm (004-300)
    
    Args:
        data: Raw ASCII frame
    
    Returns:
        Distance in meters, or None if invalid
    """
    try:
        # Remove carriage return and newline
        line = data.decode('utf-8', errors='ignore').strip()
        
        if line.startswith('R') and len(line) >= 4:
            distance_cm = int(line[1:4])
            
            # Validate range (4-300 cm)
            if 4 <= distance_cm <= 300:
                return float(distance_cm) / 100.0  # Convert to meters
    except (ValueError, AttributeError):
        pass
    
    return None


def imu_calibration_data() -> dict:
    """
    Provide default IMU calibration offsets.
    
    These are empirical values for MPU-9250.
    Should be refined through real calibration procedure.
    
    Returns:
        Dictionary of calibration offsets
    """
    return {
        'accel_offset': [0.0, 0.0, 0.0],
        'gyro_offset': [0.0, 0.0, 0.0],
        'mag_offset': [0.0, 0.0, 0.0],
        'accel_scale': [1.0, 1.0, 1.0],
        'gyro_scale': [1.0, 1.0, 1.0],
        'mag_scale': [1.0, 1.0, 1.0],
    }


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp value between min and max."""
    return max(min_val, min(max_val, value))


def get_timestamp_str() -> str:
    """Get current timestamp as ISO string."""
    return datetime.now().isoformat()


def validate_sensor_range(
    value: float,
    min_range: float,
    max_range: float,
    sensor_name: str = "sensor"
) -> Tuple[bool, str]:
    """
    Validate that sensor reading is within expected range.
    
    Args:
        value: Sensor reading
        min_range: Minimum valid value
        max_range: Maximum valid value
        sensor_name: Name for logging
    
    Returns:
        Tuple of (is_valid, message)
    """
    if min_range <= value <= max_range:
        return True, f"{sensor_name} reading valid: {value}"
    else:
        msg = f"{sensor_name} out of range: {value} (expected {min_range}-{max_range})"
        return False, msg
