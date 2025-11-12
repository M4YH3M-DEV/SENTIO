"""
Device Manager for SENTIO Hardware

Manages device connections, reconnection logic, and state tracking
for all SENTIO sensors.
"""

import logging
import threading
from typing import Dict, Optional
from enum import Enum


logger = logging.getLogger(__name__)


class DeviceState(Enum):
    """Device connection states."""
    DISCONNECTED = 0
    CONNECTING = 1
    CONNECTED = 2
    ERROR = 3
    OFFLINE = 4


class DeviceManager:
    """
    Centralized manager for hardware device connections.
    
    Tracks connection state, handles reconnection logic, and provides
    status reporting for all SENTIO sensors.
    """
    
    def __init__(self):
        """Initialize device manager."""
        self.devices: Dict[str, Dict] = {}
        self.lock = threading.Lock()
        logger.info('DeviceManager initialized')
    
    def register_device(
        self,
        device_id: str,
        device_type: str,
        connection_config: dict
    ) -> bool:
        """
        Register a new device.
        
        Args:
            device_id: Unique device identifier (e.g., 'tfmini_primary')
            device_type: Device type (e.g., 'lidar', 'imu', 'camera')
            connection_config: Connection parameters dict
        
        Returns:
            True if registered, False if already exists
        """
        with self.lock:
            if device_id in self.devices:
                logger.warning(f'Device {device_id} already registered')
                return False
            
            self.devices[device_id] = {
                'type': device_type,
                'state': DeviceState.DISCONNECTED,
                'config': connection_config,
                'error_count': 0,
                'success_count': 0,
                'last_error': None,
            }
            
            logger.info(f'Device registered: {device_id} ({device_type})')
            return True
    
    def set_device_state(self, device_id: str, state: DeviceState) -> bool:
        """
        Update device connection state.
        
        Args:
            device_id: Device identifier
            state: New state
        
        Returns:
            True if updated, False if device not found
        """
        with self.lock:
            if device_id not in self.devices:
                return False
            
            old_state = self.devices[device_id]['state']
            self.devices[device_id]['state'] = state
            
            if old_state != state:
                logger.debug(f'Device {device_id} state: {old_state.name} -> {state.name}')
            
            return True
    
    def get_device_state(self, device_id: str) -> Optional[DeviceState]:
        """Get current device state."""
        with self.lock:
            if device_id in self.devices:
                return self.devices[device_id]['state']
        return None
    
    def record_device_error(self, device_id: str, error_msg: str) -> bool:
        """
        Record an error for a device.
        
        Args:
            device_id: Device identifier
            error_msg: Error message
        
        Returns:
            True if recorded
        """
        with self.lock:
            if device_id not in self.devices:
                return False
            
            self.devices[device_id]['error_count'] += 1
            self.devices[device_id]['last_error'] = error_msg
            
            return True
    
    def record_device_success(self, device_id: str) -> bool:
        """Record a successful operation for a device."""
        with self.lock:
            if device_id not in self.devices:
                return False
            
            self.devices[device_id]['success_count'] += 1
            return True
    
    def get_device_status(self, device_id: str) -> Optional[Dict]:
        """
        Get full status of a device.
        
        Returns:
            Status dictionary or None if not found
        """
        with self.lock:
            if device_id in self.devices:
                return dict(self.devices[device_id])
        return None
    
    def get_all_device_status(self) -> Dict[str, Dict]:
        """Get status of all registered devices."""
        with self.lock:
            return {
                dev_id: dict(dev_info)
                for dev_id, dev_info in self.devices.items()
            }
    
    def unregister_device(self, device_id: str) -> bool:
        """Unregister and disconnect a device."""
        with self.lock:
            if device_id in self.devices:
                del self.devices[device_id]
                logger.info(f'Device unregistered: {device_id}')
                return True
        return False
