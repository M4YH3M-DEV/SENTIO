#!/usr/bin/env python3
"""
TFMini-S LiDAR Driver Node

Publishes distance measurements from Benewake TFMini-S LiDAR sensor.
Topic: /tfmini/distance (sensor_msgs/Range)

Protocol: UART, 115200 baud, 7-byte frames
Frame format: [0x59, 0x59, dist_lo, dist_hi, strength_lo, strength_hi, mode]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String
import serial
import threading
import time
import logging
from typing import Optional

from .utils import (
    setup_serial_connection,
    safe_serial_read,
    parse_tfmini_frame,
    validate_sensor_range,
    get_timestamp_str
)
from .device_manager import DeviceManager, DeviceState


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TFMiniNode(Node):
    """TFMini-S LiDAR driver node."""
    
    def __init__(self):
        """Initialize TFMini node."""
        super().__init__('tfmini_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate_hz', 10)
        self.declare_parameter('frame_id', 'tfmini_link')
        self.declare_parameter('min_range_m', 0.1)
        self.declare_parameter('max_range_m', 12.0)
        self.declare_parameter('sim_mode', False)
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range_m = self.get_parameter('min_range_m').value
        self.max_range_m = self.get_parameter('max_range_m').value
        self.sim_mode = self.get_parameter('sim_mode').value
        
        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.distance_pub = self.create_publisher(Range, '/tfmini/distance', qos)
        self.status_pub = self.create_publisher(String, '/tfmini/status', qos)
        
        # Device manager
        self.device_manager = DeviceManager()
        self.device_manager.register_device(
            'tfmini_primary',
            'lidar',
            {'port': self.port, 'baudrate': self.baudrate}
        )
        
        # Serial connection
        self.ser: Optional[serial.Serial] = None
        self.connected = False
        self.sim_counter = 0
        
        # Threading
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        
        # Timer for reconnection attempts
        self.reconnect_timer = self.create_timer(5.0, self._attempt_reconnect)
        
        # Initial connection
        self._connect()
        
        self.get_logger().info(
            f'TFMini Node initialized | Port: {self.port} @ {self.baudrate} baud | '
            f'Rate: {self.publish_rate_hz} Hz'
        )
    
    def _connect(self) -> bool:
        """Attempt to connect to TFMini device."""
        if self.sim_mode:
            self.get_logger().info('TFMini running in SIM mode')
            self.connected = True
            self.device_manager.set_device_state('tfmini_primary', DeviceState.CONNECTED)
            return True
        
        self.device_manager.set_device_state('tfmini_primary', DeviceState.CONNECTING)
        
        self.ser = setup_serial_connection(
            self.port,
            self.baudrate,
            timeout=0.1,
            max_retries=3
        )
        
        if self.ser:
            self.connected = True
            self.device_manager.set_device_state('tfmini_primary', DeviceState.CONNECTED)
            self._publish_status('connected', 'TFMini connected successfully')
            self.get_logger().info('TFMini connected')
            return True
        else:
            self.connected = False
            self.device_manager.set_device_state('tfmini_primary', DeviceState.ERROR)
            self._publish_status('error', f'Failed to connect to {self.port}')
            self.get_logger().error(f'Failed to connect to TFMini on {self.port}')
            return False
    
    def _attempt_reconnect(self):
        """Attempt reconnection if disconnected."""
        if not self.connected and not self.sim_mode:
            self.get_logger().info('Attempting to reconnect to TFMini...')
            self._connect()
    
    def _read_loop(self):
        """Main read loop (runs in separate thread)."""
        frame_buffer = bytearray()
        frame_size = 7
        
        while self.running:
            if not self.connected:
                time.sleep(0.1)
                continue
            
            # Simulation mode
            if self.sim_mode:
                self._publish_simulated_distance()
                time.sleep(1.0 / self.publish_rate_hz)
                continue
            
            # Real hardware mode
            try:
                byte_data = safe_serial_read(self.ser, 1, timeout_ms=50)
                
                if byte_data:
                    frame_buffer.extend(byte_data)
                    
                    # Look for frame header
                    if len(frame_buffer) >= 2:
                        # Find sync bytes
                        idx = frame_buffer.find(bytes([0x59, 0x59]))
                        
                        if idx >= 0:
                            # Remove bytes before header
                            if idx > 0:
                                frame_buffer = frame_buffer[idx:]
                            
                            # Check if we have complete frame
                            if len(frame_buffer) >= frame_size:
                                frame = bytes(frame_buffer[:frame_size])
                                frame_buffer = frame_buffer[frame_size:]
                                
                                # Parse and publish
                                self._process_frame(frame)
                
                time.sleep(0.01)  # Small sleep to prevent busy-waiting
                
            except Exception as e:
                self.get_logger().error(f'Read loop error: {str(e)}')
                self.device_manager.record_device_error('tfmini_primary', str(e))
                self.connected = False
                time.sleep(0.5)
    
    def _process_frame(self, frame: bytes):
        """
        Process a received TFMini frame.
        
        Args:
            frame: 7-byte frame
        """
        result = parse_tfmini_frame(frame)
        
        if result:
            distance_m, strength, mode = result
            
            # Validate range
            is_valid, msg = validate_sensor_range(
                distance_m,
                self.min_range_m,
                self.max_range_m,
                'TFMini distance'
            )
            
            if is_valid:
                self._publish_distance(distance_m, strength)
                self.device_manager.record_device_success('tfmini_primary')
            else:
                self.get_logger().debug(msg)
    
    def _publish_distance(self, distance_m: float, strength: int):
        """Publish distance measurement."""
        try:
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.0436  # ~2.5 degrees in radians
            msg.min_range = float(self.min_range_m)
            msg.max_range = float(self.max_range_m)
            msg.range = distance_m
            
            self.distance_pub.publish(msg)
            
            # Log debug info every 10th publish
            if int(self.get_clock().now().nanoseconds) % 10 == 0:
                self.get_logger().debug(
                    f'Distance: {distance_m:.2f}m | Strength: {strength}'
                )
        
        except Exception as e:
            self.get_logger().error(f'Publish error: {str(e)}')
    
    def _publish_simulated_distance(self):
        """Publish simulated distance for testing."""
        # Oscillate between 0.5m and 3.0m
        base = 1.75
        amplitude = 1.25
        import math
        self.sim_counter += 0.1
        distance = base + amplitude * math.sin(self.sim_counter)
        
        self._publish_distance(distance, 100)
    
    def _publish_status(self, status: str, message: str):
        """Publish status message."""
        try:
            msg = String()
            msg.data = f'status:{status} | {message}'
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Status publish error: {str(e)}')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.running = False
        
        if self.ser and self.ser.is_open:
            self.ser.close()
        
        # Cancel timers
        self.reconnect_timer.cancel()
        
        # Wait for read thread
        self.read_thread.join(timeout=2.0)
        
        self._publish_status('offline', 'TFMini node shutting down')
        self.get_logger().info('TFMini node shut down')
        
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = TFMiniNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
