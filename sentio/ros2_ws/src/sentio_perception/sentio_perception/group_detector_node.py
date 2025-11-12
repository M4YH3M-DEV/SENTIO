#!/usr/bin/env python3
"""
Group Detection Node

Detects multiple people and groups in scenes from secondary camera.

Subscribes to: /cameraB/image_raw (sensor_msgs/Image)
Publishes to: /cameraB/group_count (std_msgs/Int32)
             /cameraB/faces (detection array)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import logging
import json

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class GroupDetectorNode(Node):
    """Group/crowd detection node."""
    
    def __init__(self):
        """Initialize group detector node."""
        super().__init__('group_detector_node')
        
        # Declare parameters
        self.declare_parameter('scale_factor', 1.3)
        self.declare_parameter('min_neighbors', 5)
        self.declare_parameter('min_face_size', 30)
        self.declare_parameter('sim_mode', False)
        
        # Get parameters
        self.scale_factor = self.get_parameter('scale_factor').value
        self.min_neighbors = self.get_parameter('min_neighbors').value
        self.min_face_size = self.get_parameter('min_face_size').value
        self.sim_mode = self.get_parameter('sim_mode').value
        
        # QoS profiles
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        
        count_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.count_pub = self.create_publisher(Int32, '/cameraB/group_count', count_qos)
        self.faces_pub = self.create_publisher(String, '/cameraB/faces', count_qos)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/cameraB/image_raw',
            self.image_callback,
            image_qos
        )
        
        # Face cascade
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        
        self.bridge = CvBridge()
        self.frame_count = 0
        
        self.get_logger().info('Group Detector Node initialized')
    
    def image_callback(self, msg: Image):
        """Handle incoming images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            self.frame_count += 1
            if self.frame_count % 5 != 0:  # Process every 5th frame
                return
            
            self._process_frame(cv_image)
        
        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')
    
    def _process_frame(self, frame: np.ndarray):
        """Detect groups/faces in frame."""
        try:
            if self.sim_mode:
                self._publish_mock_detections()
                return
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=self.scale_factor,
                minNeighbors=self.min_neighbors,
                minSize=(self.min_face_size, self.min_face_size)
            )
            
            # Publish count
            count_msg = Int32()
            count_msg.data = len(faces)
            self.count_pub.publish(count_msg)
            
            # Publish faces
            faces_data = {
                'count': len(faces),
                'detections': [
                    {'x': int(x), 'y': int(y), 'w': int(w), 'h': int(h)}
                    for x, y, w, h in faces
                ]
            }
            
            faces_msg = String()
            faces_msg.data = json.dumps(faces_data)
            self.faces_pub.publish(faces_msg)
            
            if self.frame_count % 30 == 0:
                self.get_logger().debug(f'Detected {len(faces)} faces')
        
        except Exception as e:
            self.get_logger().error(f'Frame processing error: {str(e)}')
    
    def _publish_mock_detections(self):
        """Publish mock detections for SIM mode."""
        import random
        count = random.randint(0, 3)
        
        count_msg = Int32()
        count_msg.data = count
        self.count_pub.publish(count_msg)
        
        faces_data = {
            'count': count,
            'detections': [
                {'x': random.randint(50, 400), 'y': random.randint(50, 350),
                 'w': random.randint(60, 120), 'h': random.randint(60, 120)}
                for _ in range(count)
            ]
        }
        
        faces_msg = String()
        faces_msg.data = json.dumps(faces_data)
        self.faces_pub.publish(faces_msg)
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Group Detector node shut down')
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = GroupDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
