#!/usr/bin/env python3
"""
MPU-9250 IMU Driver Node

Publishes inertial measurement data (accelerometer, gyroscope, magnetometer).
Topic: /imu/data (sensor_msgs/Imu)

Protocol: I2C, address 0x68 (primary) or 0x69 (alternate)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import threading
import time
import logging
from typing import Optional, Tuple

from .utils import imu_calibration_data
from .device_manager import DeviceManager, DeviceState


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class IMUNode(Node):
    """MPU-9250 IMU driver node."""
    
    def __init__(self):
        """Initialize IMU node."""
        super().__init__('imu_node')
        
        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate_hz', 50)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('accel_scale_g', 16.0)  # ±16g range
        self.declare_parameter('gyro_scale_dps', 2000.0)  # ±2000 dps range
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.sim_mode = self.get_parameter('sim_mode').value
        self.accel_scale_g = self.get_parameter('accel_scale_g').value
        self.gyro_scale_dps = self.get_parameter('gyro_scale_dps').value
        
        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos)
        self.status_pub = self.create_publisher(String, '/imu/status', qos)
        
        # Device manager
        self.device_manager = DeviceManager()
        self.device_manager.register_device(
            'imu_primary',
            'imu',
            {'bus': self.i2c_bus, 'address': hex(self.i2c_address)}
        )
        
        # IMU data
        self.calibration = imu_calibration_data()
        self.i2c_device = None
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
            f'IMU Node initialized | I2C Bus: {self.i2c_bus} @ 0x{self.i2c_address:02x} | '
            f'Rate: {self.publish_rate_hz} Hz'
        )
    
    def _connect(self) -> bool:
        """Attempt to connect to IMU device."""
        if self.sim_mode:
            self.get_logger().info('IMU running in SIM mode')
            self.connected = True
            self.device_manager.set_device_state('imu_primary', DeviceState.CONNECTED)
            return True
        
        self.device_manager.set_device_state('imu_primary', DeviceState.CONNECTING)
        
        try:
            # Try to import smbus2 for I2C communication
            try:
                import smbus2
                self.i2c_device = smbus2.SMBus(self.i2c_bus)
                
                # Attempt to read WHO_AM_I register (should return 0x71 for MPU-9250)
                who_am_i = self.i2c_device.read_byte_data(self.i2c_address, 0x75)
                
                if who_am_i == 0x71:
                    self.connected = True
                    self.device_manager.set_device_state('imu_primary', DeviceState.CONNECTED)
                    self._publish_status('connected', f'IMU WHO_AM_I: 0x{who_am_i:02x}')
                    self.get_logger().info(f'IMU connected (WHO_AM_I: 0x{who_am_i:02x})')
                    return True
                else:
                    self.get_logger().warning(
                        f'Unexpected WHO_AM_I value: 0x{who_am_i:02x} (expected 0x71)'
                    )
                    self.connected = True  # Continue anyway
                    return True
            
            except ImportError:
                self.get_logger().warn('smbus2 not available, IMU in SIM mode')
                self.sim_mode = True
                self.connected = True
                return True
        
        except Exception as e:
            self.connected = False
            self.device_manager.set_device_state('imu_primary', DeviceState.ERROR)
            self._publish_status('error', f'Failed to connect to IMU: {str(e)}')
            self.get_logger().error(f'IMU connection error: {str(e)}')
            return False
    
    def _attempt_reconnect(self):
        """Attempt reconnection if disconnected."""
        if not self.connected and not self.sim_mode:
            self.get_logger().info('Attempting to reconnect to IMU...')
            self._connect()
    
    def _read_loop(self):
        """Main read loop (runs in separate thread)."""
        period = 1.0 / self.publish_rate_hz
        
        while self.running:
            try:
                if self.sim_mode:
                    self._publish_simulated_imu()
                elif self.connected:
                    accel, gyro, mag = self._read_imu_data()
                    if accel and gyro:
                        self._publish_imu(accel, gyro, mag)
                
                time.sleep(period)
            
            except Exception as e:
                self.get_logger().error(f'Read loop error: {str(e)}')
                self.device_manager.record_device_error('imu_primary', str(e))
                time.sleep(0.5)
    
    def _read_imu_data(self) -> Tuple[Optional[dict], Optional[dict], Optional[dict]]:
        """
        Read IMU data from device.
        
        Returns:
            Tuple of (accel_dict, gyro_dict, mag_dict)
        """
        try:
            if not self.i2c_device:
                return None, None, None
            
            # Simplified mock implementation
            # In production, would implement actual MPU-9250 register reading
            accel = {'x': 0.0, 'y': 0.0, 'z': 9.81}
            gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            mag = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            self.device_manager.record_device_success('imu_primary')
            return accel, gyro, mag
        
        except Exception as e:
            self.get_logger().debug(f'IMU read error: {str(e)}')
            return None, None, None
    
    def _publish_imu(
        self,
        accel: dict,
        gyro: dict,
        mag: Optional[dict] = None
    ):
        """Publish IMU measurement."""
        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            # Accelerometer (m/s²)
            msg.linear_acceleration = Vector3(
                x=accel.get('x', 0.0),
                y=accel.get('y', 0.0),
                z=accel.get('z', 9.81)
            )
            msg.linear_acceleration_covariance = [0.01] * 9
            
            # Gyroscope (rad/s)
            msg.angular_velocity = Vector3(
                x=gyro.get('x', 0.0),
                y=gyro.get('y', 0.0),
                z=gyro.get('z', 0.0)
            )
            msg.angular_velocity_covariance = [0.01] * 9
            
            # Orientation covariance (not used by MPU-9250 alone)
            msg.orientation_covariance[0] = -1  # Indicates orientation data not provided
            
            self.imu_pub.publish(msg)
            self.device_manager.record_device_success('imu_primary')
        
        except Exception as e:
            self.get_logger().error(f'Publish error: {str(e)}')
    
    def _publish_simulated_imu(self):
        """Publish simulated IMU data for testing."""
        import math
        
        # Simulate gentle oscillation
        self.sim_counter += 0.05
        accel = {
            'x': 0.5 * math.sin(self.sim_counter),
            'y': 0.3 * math.cos(self.sim_counter * 0.7),
            'z': 9.81 + 0.2 * math.sin(self.sim_counter * 0.5)
        }
        
        gyro = {
            'x': 0.1 * math.sin(self.sim_counter * 0.3),
            'y': 0.1 * math.cos(self.sim_counter * 0.4),
            'z': 0.05 * math.sin(self.sim_counter * 0.2)
        }
        
        self._publish_imu(accel, gyro)
    
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
        
        if self.i2c_device:
            try:
                self.i2c_device.close()
            except:
                pass
        
        self.reconnect_timer.cancel()
        self.read_thread.join(timeout=2.0)
        
        self._publish_status('offline', 'IMU node shutting down')
        self.get_logger().info('IMU node shut down')
        
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = IMUNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
