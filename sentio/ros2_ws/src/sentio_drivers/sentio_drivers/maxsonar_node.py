#!/usr/bin/env python3
"""
MaxSonar Ultrasonic Range Sensor Driver Node

Publishes distance measurements from MaxSonar ultrasonic sensor.
Topic: /maxsonar/distance (sensor_msgs/Range)

Protocol: UART, variable baud (typically 9600), ASCII format
Output format: 'Rxxx\r' where xxx is distance in cm (004-300)
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
    parse_maxsonar_ascii,
    validate_sensor_range,
)
from .device_manager import DeviceManager, DeviceState


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MaxSonarNode(Node):
    """MaxSonar ultrasonic range sensor driver node."""
    
    def __init__(self):
        """Initialize MaxSonar node."""
        super().__init__('maxsonar_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('publish_rate_hz', 20)
        self.declare_parameter('frame_id', 'maxsonar_link')
        self.declare_parameter('min_range_m', 0.04)
        self.declare_parameter('max_range_m', 3.0)
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
        self.distance_pub = self.create_publisher(Range, '/maxsonar/distance', qos)
        self.status_pub = self.create_publisher(String, '/maxsonar/status', qos)
        
        # Device manager
        self.device_manager = DeviceManager()
        self.device_manager.register_device(
            'maxsonar_primary',
            'ultrasonic',
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
        
        # Reconnection timer
        self.reconnect_timer = self.create_timer(5.0, self._attempt_reconnect)
        
        # Initial connection
        self._connect()
        
        self.get_logger().info(
            f'MaxSonar Node initialized | Port: {self.port} @ {self.baudrate} baud | '
            f'Rate: {self.publish_rate_hz} Hz'
        )
    
    def _connect(self) -> bool:
        """Attempt to connect to MaxSonar device."""
        if self.sim_mode:
            self.get_logger().info('MaxSonar running in SIM mode')
            self.connected = True
            self.device_manager.set_device_state('maxsonar_primary', DeviceState.CONNECTED)
            return True
        
        self.device_manager.set_device_state('maxsonar_primary', DeviceState.CONNECTING)
        
        self.ser = setup_serial_connection(
            self.port,
            self.baudrate,
            timeout=0.2,
            max_retries=3
        )
        
        if self.ser:
            self.connected = True
            self.device_manager.set_device_state('maxsonar_primary', DeviceState.CONNECTED)
            self._publish_status('connected', 'MaxSonar connected successfully')
            self.get_logger().info('MaxSonar connected')
            return True
        else:
            self.connected = False
            self.device_manager.set_device_state('maxsonar_primary', DeviceState.ERROR)
            self._publish_status('error', f'Failed to connect to {self.port}')
            self.get_logger().error(f'Failed to connect to MaxSonar on {self.port}')
            return False
    
    def _attempt_reconnect(self):
        """Attempt reconnection if disconnected."""
        if not self.connected and not self.sim_mode:
            self.get_logger().info('Attempting to reconnect to MaxSonar...')
            self._connect()
    
    def _read_loop(self):
        """Main read loop (runs in separate thread)."""
        line_buffer = bytearray()
        
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
                    line_buffer.extend(byte_data)
                    
                    # Look for end-of-line marker
                    if b'\r' in line_buffer:
                        line_end = line_buffer.find(b'\r')
                        line = bytes(line_buffer[:line_end])
                        line_buffer = line_buffer[line_end + 1:]
                        
                        # Parse and publish
                        self._process_line(line)
                
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error(f'Read loop error: {str(e)}')
                self.device_manager.record_device_error('maxsonar_primary', str(e))
                self.connected = False
                time.sleep(0.5)
    
    def _process_line(self, line: bytes):
        """
        Process a received MaxSonar line.
        
        Args:
            line: ASCII line (e.g., b'R125')
        """
        distance_m = parse_maxsonar_ascii(line)
        
        if distance_m is not None:
            is_valid, msg = validate_sensor_range(
                distance_m,
                self.min_range_m,
                self.max_range_m,
                'MaxSonar distance'
            )
            
            if is_valid:
                self._publish_distance(distance_m)
                self.device_manager.record_device_success('maxsonar_primary')
            else:
                self.get_logger().debug(msg)
    
    def _publish_distance(self, distance_m: float):
        """Publish distance measurement."""
        try:
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.2618  # ~15 degrees
            msg.min_range = float(self.min_range_m)
            msg.max_range = float(self.max_range_m)
            msg.range = distance_m
            
            self.distance_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Publish error: {str(e)}')
    
    def _publish_simulated_distance(self):
        """Publish simulated distance for testing."""
        import math
        base = 1.5
        amplitude = 1.0
        self.sim_counter += 0.15
        distance = base + amplitude * math.sin(self.sim_counter)
        
        self._publish_distance(distance)
    
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
        
        self.reconnect_timer.cancel()
        self.read_thread.join(timeout=2.0)
        
        self._publish_status('offline', 'MaxSonar node shutting down')
        self.get_logger().info('MaxSonar node shut down')
        
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = MaxSonarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
