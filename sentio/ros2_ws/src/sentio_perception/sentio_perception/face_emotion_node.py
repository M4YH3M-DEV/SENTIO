#!/usr/bin/env python3
"""
Face Emotion Detection Node

Detects facial expressions and classifies emotions in real-time.

Subscribes to: /camera/image_raw (sensor_msgs/Image)
Publishes to: /affect/vision (sentio_msgs/Affect)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import logging
import threading
from typing import Optional

from sentio_msgs.msg import Affect
from .model_loader import ModelLoader
from .emotion_utils import EmotionMapper, SmoothingFilter


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FaceEmotionNode(Node):
    """Face emotion detection node."""
    
    def __init__(self):
        """Initialize face emotion node."""
        super().__init__('face_emotion_node')
        
        # Declare parameters
        self.declare_parameter('models_dir', '/root/aether_sentio_ws/models')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('detection_frequency', 5)
        self.declare_parameter('sim_mode', False)
        
        # Get parameters
        self.models_dir = self.get_parameter('models_dir').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.detection_frequency = self.get_parameter('detection_frequency').value
        self.sim_mode = self.get_parameter('sim_mode').value
        
        # QoS profiles
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        
        affect_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.affect_pub = self.create_publisher(Affect, '/affect/vision', affect_qos)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            image_qos
        )
        
        # Model and utilities
        self.model_loader = ModelLoader(self.models_dir)
        self.face_model = self.model_loader.load_model('face_emotion')
        self.bridge = CvBridge()
        self.smoothing_filter = SmoothingFilter(self.smoothing_alpha)
        
        # Face cascade for detection
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        
        # State tracking
        self.frame_count = 0
        self.last_emotion = 'neutral'
        self.last_confidence = 0.0
        self.lock = threading.Lock()
        
        self.get_logger().info(
            f'Face Emotion Node initialized | '
            f'Models: {self.models_dir} | Threshold: {self.confidence_threshold}'
        )
    
    def image_callback(self, msg: Image):
        """
        Handle incoming camera frames.
        
        Args:
            msg: Image message
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process frame based on frequency
            self.frame_count += 1
            if self.frame_count % (30 // self.detection_frequency) != 0:
                return
            
            # Detect and classify emotion
            self._process_frame(cv_image)
        
        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')
    
    def _process_frame(self, frame: np.ndarray):
        """
        Process a single frame for emotion detection.
        
        Args:
            frame: OpenCV image (BGR format)
        """
        try:
            # Detect faces
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.3,
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            if len(faces) == 0:
                # No faces detected
                self._publish_affect('neutral', 0.0, 'no_face_detected')
                return
            
            # Use largest face
            face = max(faces, key=lambda f: f[2] * f[3])
            x, y, w, h = face
            
            # Extract face ROI
            face_roi = frame[y:y+h, x:x+w]
            
            # Preprocess for model
            face_processed = self._preprocess_face(face_roi)
            
            if face_processed is None:
                return
            
            # Inference
            if self.sim_mode:
                emotion, confidence = self._mock_inference()
            else:
                emotion, confidence = self._run_inference(face_processed)
            
            # Publish result
            self._publish_affect(emotion, confidence, f'face_detected:{len(faces)}')
        
        except Exception as e:
            self.get_logger().error(f'Frame processing error: {str(e)}')
    
    def _preprocess_face(self, face_roi: np.ndarray) -> Optional[np.ndarray]:
        """
        Preprocess face ROI for model inference.
        
        Args:
            face_roi: Face region of interest
        
        Returns:
            Preprocessed tensor or None
        """
        try:
            # Resize to 224x224 (typical for emotion models)
            resized = cv2.resize(face_roi, (224, 224))
            
            # Convert BGR to RGB
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            
            # Normalize to [0, 1]
            normalized = rgb.astype(np.float32) / 255.0
            
            # Transpose to CHW format
            tensor = normalized.transpose(2, 0, 1)
            
            # Add batch dimension
            batch = np.expand_dims(tensor, axis=0)
            
            return batch
        
        except Exception as e:
            self.get_logger().error(f'Face preprocessing error: {str(e)}')
            return None
    
    def _run_inference(self, face_tensor: np.ndarray) -> tuple:
        """
        Run emotion inference on face tensor.
        
        Args:
            face_tensor: Preprocessed face tensor
        
        Returns:
            Tuple of (emotion_label, confidence)
        """
        try:
            if self.face_model is None:
                return 'neutral', 0.0
            
            # Prepare input
            input_name = self.face_model.get_inputs()[0].name
            input_feed = {input_name: face_tensor}
            
            # Run inference
            outputs = self.face_model.run(None, input_feed)
            logits = outputs[0][0]
            
            # Get emotion with highest score
            emotion_labels = ['neutral', 'happy', 'sad', 'angry', 
                            'fearful', 'disgusted', 'surprised']
            
            emotion_idx = np.argmax(logits)
            emotion = emotion_labels[emotion_idx]
            confidence = float(np.exp(logits[emotion_idx]) / np.sum(np.exp(logits)))
            
            return emotion, confidence
        
        except Exception as e:
            self.get_logger().error(f'Inference error: {str(e)}')
            return 'neutral', 0.0
    
    def _mock_inference(self) -> tuple:
        """Mock inference for SIM mode."""
        import random
        emotions = ['neutral', 'happy', 'sad', 'angry', 'fearful', 'disgusted', 'surprised']
        emotion = random.choice(emotions)
        confidence = random.uniform(0.6, 0.95)
        return emotion, confidence
    
    def _publish_affect(self, emotion: str, confidence: float, source: str):
        """
        Publish emotion as Affect message.
        
        Args:
            emotion: Emotion label
            confidence: Confidence score
            source: Source identifier
        """
        try:
            # Apply smoothing
            valence, arousal = EmotionMapper.emotion_to_valence_arousal(
                emotion,
                confidence
            )
            
            smoothed_valence, smoothed_arousal = self.smoothing_filter.smooth(
                valence, arousal
            )
            
            # Create message
            msg = Affect()
            msg.valence = smoothed_valence
            msg.arousal = smoothed_arousal
            msg.label = EmotionMapper.emotion_to_label(emotion)
            msg.label_text = f'{emotion}:vision:{source}'
            
            self.affect_pub.publish(msg)
            
            # Log periodically
            if self.frame_count % 30 == 0:
                self.get_logger().debug(
                    f'Vision emotion: {emotion} ({confidence:.2f}) -> '
                    f'VA({smoothed_valence:.2f}, {smoothed_arousal:.2f})'
                )
        
        except Exception as e:
            self.get_logger().error(f'Affect publish error: {str(e)}')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Face Emotion node shut down')
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = FaceEmotionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
