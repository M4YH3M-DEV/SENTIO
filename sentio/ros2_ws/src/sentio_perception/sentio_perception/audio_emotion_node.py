#!/usr/bin/env python3
"""
Audio Emotion Detection Node

Analyzes voice tone and emotional content from audio streams.
Includes optional speech-to-text with faster-whisper.

Subscribes to: /audio/data (audio chunks) or internal microphone
Publishes to: /affect/audio (sentio_msgs/Affect)
             /audio/transcription (std_msgs/String) [if speech enabled]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import numpy as np
import logging
import threading
import time
from typing import Optional, Deque
from collections import deque

from sentio_msgs.msg import Affect
from .model_loader import ModelLoader
from .audio_processor import AudioProcessor
from .emotion_utils import EmotionMapper, SmoothingFilter

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class AudioEmotionNode(Node):
    """Audio emotion detection node with optional speech-to-text."""
    
    def __init__(self):
        """Initialize audio emotion node."""
        super().__init__('audio_emotion_node')
        
        # Declare parameters
        self.declare_parameter('models_dir', '/root/aether_sentio_ws/models')
        self.declare_parameter('sample_rate', 22050)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('buffer_size_s', 2.0)
        self.declare_parameter('sim_mode', False)
        
        # Speech-to-text parameters
        self.declare_parameter('enable_speech', True)
        self.declare_parameter('whisper_model', 'base')
        self.declare_parameter('whisper_device', 'cpu')
        self.declare_parameter('whisper_compute_type', 'int8')
        
        # Get parameters
        self.models_dir = self.get_parameter('models_dir').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.buffer_size_s = self.get_parameter('buffer_size_s').value
        self.sim_mode = self.get_parameter('sim_mode').value
        
        self.enable_speech = self.get_parameter('enable_speech').value
        self.whisper_model = self.get_parameter('whisper_model').value
        self.whisper_device = self.get_parameter('whisper_device').value
        self.whisper_compute_type = self.get_parameter('whisper_compute_type').value
        
        # QoS profile
        affect_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.affect_pub = self.create_publisher(Affect, '/affect/audio', affect_qos)
        self.transcription_pub = self.create_publisher(String, '/audio/transcription', 10)
        
        # Model and utilities
        self.model_loader = ModelLoader(self.models_dir)
        self.audio_model = self.model_loader.load_model('audio_affect')
        
        # Initialize audio processor with faster-whisper
        self.audio_processor = AudioProcessor(
            sample_rate=self.sample_rate,
            enable_speech=self.enable_speech,
            whisper_model=self.whisper_model,
            whisper_device=self.whisper_device,
            whisper_compute_type=self.whisper_compute_type
        )
        
        self.smoothing_filter = SmoothingFilter(self.smoothing_alpha)
        
        # Audio buffer
        self.buffer_size = int(self.buffer_size_s * self.sample_rate)
        self.audio_buffer: Deque[float] = deque(maxlen=self.buffer_size)
        self.buffer_lock = threading.Lock()
        
        # State tracking
        self.running = True
        self.last_transcription_time = 0.0
        self.transcription_interval = 5.0  # Transcribe every 5 seconds
        
        # Analysis thread
        self.analysis_thread = threading.Thread(
            target=self._analysis_loop,
            daemon=True
        )
        self.analysis_thread.start()
        
        self.get_logger().info(
            f'Audio Emotion Node initialized | '
            f'Sample rate: {self.sample_rate} Hz | Buffer: {self.buffer_size_s}s | '
            f'Speech-to-text: {"enabled" if self.enable_speech else "disabled"}'
        )
        
        if self.enable_speech:
            self.get_logger().info(
                f'faster-whisper: {self.whisper_model} on {self.whisper_device} '
                f'({self.whisper_compute_type})'
            )
    
    def _analysis_loop(self):
        """Continuous audio analysis loop."""
        analysis_interval = 1.0  # Analyze every 1 second
        
        while self.running:
            try:
                if self.sim_mode:
                    self._publish_mock_affect()
                else:
                    self._analyze_audio()
                
                # Periodic transcription
                if self.enable_speech and not self.sim_mode:
                    current_time = time.time()
                    if current_time - self.last_transcription_time > self.transcription_interval:
                        self._transcribe_audio()
                        self.last_transcription_time = current_time
                
                time.sleep(analysis_interval)
            
            except Exception as e:
                self.get_logger().error(f'Analysis loop error: {str(e)}')
                time.sleep(1.0)
    
    def _analyze_audio(self):
        """Analyze audio buffer for emotion."""
        try:
            with self.buffer_lock:
                if len(self.audio_buffer) < self.buffer_size // 2:
                    # Not enough audio data
                    return
                
                # Convert buffer to numpy array
                audio_array = np.array(list(self.audio_buffer), dtype=np.float32)
            
            # Extract features
            features = self.audio_processor.extract_features(audio_array)
            
            if features is None or len(features) == 0:
                return
            
            # Run inference
            emotion, confidence = self._run_inference(features)
            
            # Publish result
            self._publish_affect(emotion, confidence)
        
        except Exception as e:
            self.get_logger().error(f'Audio analysis error: {str(e)}')
    
    def _transcribe_audio(self):
        """Transcribe audio buffer to text using faster-whisper."""
        try:
            with self.buffer_lock:
                if len(self.audio_buffer) < self.buffer_size // 4:
                    return
                
                audio_array = np.array(list(self.audio_buffer), dtype=np.float32)
            
            # Transcribe using faster-whisper
            transcription = self.audio_processor.transcribe_audio_array(
                audio_array,
                beam_size=5
            )
            
            if transcription and len(transcription.strip()) > 0:
                # Publish transcription
                msg = String()
                msg.data = transcription
                self.transcription_pub.publish(msg)
                
                self.get_logger().info(f'Transcription: "{transcription[:50]}..."')
        
        except Exception as e:
            self.get_logger().error(f'Transcription error: {str(e)}')
    
    def _run_inference(self, features: np.ndarray) -> tuple:
        """
        Run emotion inference on audio features.
        
        Args:
            features: Audio feature vector
        
        Returns:
            Tuple of (emotion_label, confidence)
        """
        try:
            if self.audio_model is None:
                return 'neutral', 0.0
            
            # Prepare input
            input_name = self.audio_model.get_inputs().name
            
            # Ensure correct shape
            if len(features.shape) == 1:
                features = features.reshape(1, -1)
            
            input_feed = {input_name: features.astype(np.float32)}
            
            # Run inference
            outputs = self.audio_model.run(None, input_feed)
            logits = outputs
            
            # Get emotion with highest score
            emotion_labels = ['neutral', 'happy', 'sad', 'angry']
            
            emotion_idx = np.argmax(logits)
            emotion = emotion_labels[emotion_idx]
            confidence = float(np.exp(logits[emotion_idx]) / np.sum(np.exp(logits)))
            
            return emotion, confidence
        
        except Exception as e:
            self.get_logger().error(f'Inference error: {str(e)}')
            return 'neutral', 0.0
    
    def _publish_mock_affect(self):
        """Publish mock affect for SIM mode."""
        import random
        emotions = ['neutral', 'happy', 'sad', 'angry']
        emotion = random.choice(emotions)
        confidence = random.uniform(0.6, 0.9)
        self._publish_affect(emotion, confidence)
    
    def _publish_affect(self, emotion: str, confidence: float):
        """
        Publish emotion as Affect message.
        
        Args:
            emotion: Emotion label
            confidence: Confidence score
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
            msg.label_text = f'{emotion}:audio'
            
            self.affect_pub.publish(msg)
            
            self.get_logger().debug(
                f'Audio emotion: {emotion} ({confidence:.2f}) -> '
                f'VA({smoothed_valence:.2f}, {smoothed_arousal:.2f})'
            )
        
        except Exception as e:
            self.get_logger().error(f'Affect publish error: {str(e)}')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.running = False
        self.analysis_thread.join(timeout=5.0)
        self.get_logger().info('Audio Emotion node shut down')
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = AudioEmotionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()