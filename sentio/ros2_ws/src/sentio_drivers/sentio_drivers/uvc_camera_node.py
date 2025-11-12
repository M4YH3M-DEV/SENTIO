#!/usr/bin/env python3
"""
USB Camera (UVC) Driver Node

Publishes camera images from standard USB cameras.
Topic: /camera/image_raw (sensor_msgs/Image)

Uses OpenCV for camera capture and image publishing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import threading
import time
import logging
from typing import Optional

from .device_manager import DeviceManager, DeviceState


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class UVCCameraNode(Node):
    """USB camera driver node."""
    
    def __init__(self):
        """Initialize camera node."""
        super().__init__('uvc_camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('sim_mode', False)
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.sim_mode = self.get_parameter('sim_mode').value
        
        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos)
        self.status_pub = self.create_publisher(String, '/camera/status', qos)
        
        # Device manager
        self.device_manager = DeviceManager()
        self.device_manager.register_device(
            'camera_primary',
            'camera',
            {'index': self.camera_index, 'resolution': f'{self.frame_width}x{self.frame_height}'}
        )
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Camera object
        self.cap: Optional[cv2.VideoCapture] = None
        self.connected = False
        
        # Threading
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        
        # Reconnection timer
        self.reconnect_timer = self.create_timer(5.0, self._attempt_reconnect)
        
        # Initial connection
        self._connect()
        
        self.get_logger().info(
            f'Camera Node initialized | Index: {self.camera_index} | '
            f'Resolution: {self.frame_width}x{self.frame_height} | FPS: {self.fps}'
        )
    
    def _connect(self) -> bool:
        """Attempt to connect to camera device."""
        if self.sim_mode:
            self.get_logger().info('Camera running in SIM mode')
            self.connected = True
            self.device_manager.set_device_state('camera_primary', DeviceState.CONNECTED)
            return True
        
        self.device_manager.set_device_state('camera_primary', DeviceState.CONNECTING)
        
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if self.cap.isOpened():
                # Set camera properties
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Single frame buffer
                
                self.connected = True
                self.device_manager.set_device_state('camera_primary', DeviceState.CONNECTED)
                self._publish_status('connected', f'Camera {self.camera_index} opened')
                self.get_logger().info(f'Camera {self.camera_index} connected')
                return True
            else:
                self.connected = False
                self.device_manager.set_device_state('camera_primary', DeviceState.ERROR)
                self._publish_status('error', f'Failed to open camera {self.camera_index}')
                self.get_logger().error(f'Failed to open camera {self.camera_index}')
                return False
        
        except Exception as e:
            self.connected = False
            self.device_manager.set_device_state('camera_primary', DeviceState.ERROR)
            self._publish_status('error', f'Camera connection error: {str(e)}')
            self.get_logger().error(f'Camera connection error: {str(e)}')
            return False
    
    def _attempt_reconnect(self):
        """Attempt reconnection if disconnected."""
        if not self.connected and not self.sim_mode:
            self.get_logger().info(f'Attempting to reconnect to camera {self.camera_index}...')
            self._connect()
    
    def _capture_loop(self):
        """Main capture loop (runs in separate thread)."""
        frame_count = 0
        
        while self.running:
            try:
                if self.sim_mode:
                    self._publish_simulated_frame()
                elif self.connected and self.cap:
                    ret, frame = self.cap.read()
                    
                    if ret:
                        self._publish_frame(frame)
                        self.device_manager.record_device_success('camera_primary')
                        frame_count += 1
                    else:
                        self.get_logger().warning('Failed to read frame from camera')
                        self.connected = False
                
                # Control frame rate
                time.sleep(1.0 / self.fps)
            
            except Exception as e:
                self.get_logger().error(f'Capture loop error: {str(e)}')
                self.device_manager.record_device_error('camera_primary', str(e))
                self.connected = False
                time.sleep(1.0)
    
    def _publish_frame(self, cv_frame):
        """Publish camera frame."""
        try:
            # Convert OpenCV frame to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = self.frame_id
            
            self.image_pub.publish(ros_image)
        
        except Exception as e:
            self.get_logger().error(f'Publish error: {str(e)}')
    
    def _publish_simulated_frame(self):
        """Publish simulated camera frame for testing."""
        import numpy as np
        
        # Create a simple test pattern
        frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        
        # Add some pattern
        frame[:, :] = [100, 100, 100]
        
        # Add a moving circle
        import math
        angle = (time.time() * 2 * math.pi) % (2 * math.pi)
        cx = int(self.frame_width / 2 + 150 * math.cos(angle))
        cy = int(self.frame_height / 2 + 150 * math.sin(angle))
        
        cv2.circle(frame, (cx, cy), 30, (0, 255, 0), -1)
        
        self._publish_frame(frame)
    
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
        
        if self.cap:
            self.cap.release()
        
        self.reconnect_timer.cancel()
        self.capture_thread.join(timeout=2.0)
        
        self._publish_status('offline', 'Camera node shutting down')
        self.get_logger().info('Camera node shut down')
        
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = UVCCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
