#!/usr/bin/env python3
"""
SENTIO TTS Bridge Node

Main ROS 2 node for text-to-speech synthesis using Piper TTS server.

Subscribes to: /tts_text (std_msgs/String) with format "text:[text]|voice:[voice]|emotion:[emotion]"
Publishes to: /tts_status (std_msgs/String) with format "done:[filepath]" or "error:[message]"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import threading
import time
import logging
import sys
from typing import Optional
from datetime import datetime

from .piper_client import PiperClient, PiperClientError
from .audio_manager import AudioManager
from .voice_config import VoiceConfig


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SentioTTSBridgeNode(Node):
    """
    SENTIO TTS Bridge ROS 2 node.
    
    Bridges text input requests to Piper TTS HTTP server and publishes
    resulting audio files and status information.
    """
    
    def __init__(self):
        """Initialize TTS bridge node."""
        super().__init__('sentio_tts_bridge_node')
        
        # Declare parameters
        self.declare_parameter('piper_url', 'http://localhost:5000')
        self.declare_parameter('piper_timeout_s', 30.0)
        self.declare_parameter('piper_max_retries', 3)
        self.declare_parameter('audio_base_dir', '/tmp/sentio_tts')
        self.declare_parameter('default_voice', 'sentio_neutral')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('health_check_interval_s', 10.0)
        self.declare_parameter('sim_mode', False)
        
        # Get parameters
        self.piper_url = self.get_parameter('piper_url').value
        self.piper_timeout = self.get_parameter('piper_timeout_s').value
        self.piper_max_retries = self.get_parameter('piper_max_retries').value
        self.audio_base_dir = self.get_parameter('audio_base_dir').value
        self.default_voice = self.get_parameter('default_voice').value
        self.queue_size = self.get_parameter('queue_size').value
        self.health_check_interval = self.get_parameter('health_check_interval_s').value
        self.sim_mode = self.get_parameter('sim_mode').value
        
        # QoS profiles
        text_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.queue_size
        )
        
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.queue_size
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/tts_status', status_qos)
        self.diagnostics_pub = self.create_publisher(String, '/tts_diagnostics', status_qos)
        
        # Subscribers
        self.text_sub = self.create_subscription(
            String,
            '/tts_text',
            self.text_callback,
            text_qos
        )
        
        # Initialize managers
        self.piper_client = PiperClient(
            server_url=self.piper_url,
            timeout=self.piper_timeout,
            max_retries=self.piper_max_retries
        )
        
        self.audio_manager = AudioManager(self.audio_base_dir)
        self.voice_config = VoiceConfig()
        self.voice_config.set_current_voice(self.default_voice)
        
        # State tracking
        self.synthesis_queue = []
        self.queue_lock = threading.Lock()
        self.running = True
        self.piper_healthy = False
        self.synthesis_count = 0
        self.error_count = 0
        
        # Worker thread for asynchronous synthesis
        self.worker_thread = threading.Thread(target=self._synthesis_worker, daemon=True)
        self.worker_thread.start()
        
        # Health check timer
        self.health_timer = self.create_timer(
            self.health_check_interval,
            self._health_check
        )
        
        # Diagnostics timer
        self.diagnostics_timer = self.create_timer(30.0, self._publish_diagnostics)
        
        # Initial health check
        self._health_check()
        
        self.get_logger().info(
            f'SENTIO TTS Bridge initialized | Piper: {self.piper_url} | '
            f'Audio dir: {self.audio_base_dir} | SIM: {self.sim_mode}'
        )
    
    def text_callback(self, msg: String):
        """
        Handle incoming text requests.
        
        Expected format: "text:Hello world|voice:sentio_friendly|emotion:happy"
        Or simple format: "Hello world" (uses default voice)
        
        Args:
            msg: String message with text request
        """
        try:
            request_text = msg.data.strip()
            
            if not request_text:
                self.get_logger().warning('Received empty text request')
                return
            
            # Parse message
            voice = self.default_voice
            emotion = None
            
            if '|' in request_text:
                # Structured format
                parts = {}
                for part in request_text.split('|'):
                    if ':' in part:
                        key, val = part.split(':', 1)
                        parts[key.strip()] = val.strip()
                
                text = parts.get('text', '')
                voice = parts.get('voice', self.default_voice)
                emotion = parts.get('emotion')
                
                if not text:
                    self.get_logger().warning('No text in structured request')
                    return
            else:
                # Simple format
                text = request_text
            
            # If emotion provided, select matching voice
            if emotion:
                selected = self.voice_config.select_emotion_voice(emotion)
                if selected:
                    voice = selected
            
            # Queue synthesis request
            with self.queue_lock:
                if len(self.synthesis_queue) < self.queue_size:
                    self.synthesis_queue.append({
                        'text': text,
                        'voice': voice,
                        'timestamp': self.get_clock().now()
                    })
                    self.get_logger().debug(
                        f'TTS queued: "{text[:50]}" | voice: {voice}'
                    )
                else:
                    self.get_logger().warning('TTS queue full, discarding request')
                    self._publish_status('error', 'Queue full')
        
        except Exception as e:
            self.get_logger().error(f'Text callback error: {str(e)}')
            self._publish_status('error', f'Parse error: {str(e)}')
    
    def _synthesis_worker(self):
        """Worker thread for asynchronous TTS synthesis."""
        while self.running:
            try:
                # Get next request
                request = None
                with self.queue_lock:
                    if self.synthesis_queue:
                        request = self.synthesis_queue.pop(0)
                
                if request:
                    self._process_synthesis_request(request)
                else:
                    time.sleep(0.1)
            
            except Exception as e:
                self.get_logger().error(f'Worker thread error: {str(e)}')
                time.sleep(1.0)
    
    def _process_synthesis_request(self, request: dict):
        """
        Process a single TTS synthesis request.
        
        Args:
            request: Dictionary with 'text', 'voice', 'timestamp'
        """
        text = request['text']
        voice = request['voice']
        
        try:
            # Check cache first
            cached_path = self.audio_manager.get_cached_audio(text, voice)
            if cached_path:
                self.get_logger().info(f'Using cached audio: {cached_path}')
                self._publish_status('done', cached_path)
                self.synthesis_count += 1
                return
            
            # Simulation mode
            if self.sim_mode:
                self._publish_simulated_audio(text, voice)
                self.synthesis_count += 1
                return
            
            # Check Piper health
            if not self.piper_healthy:
                self.get_logger().error('Piper server not healthy')
                self._publish_status('error', 'Piper server unavailable')
                self.error_count += 1
                return
            
            # Synthesize with Piper
            self.get_logger().info(f'Synthesizing: "{text[:50]}..." with voice: {voice}')
            
            success, audio_data, error_msg = self.piper_client.synthesize(
                text=text,
                voice=voice,
                **self.voice_config.get_synthesis_params()
            )
            
            if success and audio_data:
                # Save audio file
                filepath = self.audio_manager.save_audio(audio_data, text, voice)
                
                if filepath:
                    self.get_logger().info(f'Synthesis complete: {filepath}')
                    self._publish_status('done', filepath)
                    self.synthesis_count += 1
                else:
                    self.get_logger().error('Failed to save audio')
                    self._publish_status('error', 'Failed to save audio')
                    self.error_count += 1
            else:
                self.get_logger().error(f'Synthesis failed: {error_msg}')
                self._publish_status('error', error_msg or 'Unknown error')
                self.error_count += 1
        
        except Exception as e:
            self.get_logger().error(f'Synthesis error: {str(e)}')
            self._publish_status('error', str(e))
            self.error_count += 1
    
    def _publish_simulated_audio(self, text: str, voice: str):
        """Publish simulated audio for testing."""
        try:
            # Generate dummy audio data (WAV header + silence)
            import struct
            
            sample_rate = 22050
            duration_s = len(text) * 0.05  # Estimate 50ms per character
            num_samples = int(sample_rate * duration_s)
            
            # WAV header
            audio_data = bytearray()
            audio_data.extend(b'RIFF')
            
            # File size (will update)
            file_size = 36 + num_samples * 2
            audio_data.extend(struct.pack('<I', file_size))
            
            audio_data.extend(b'WAVE')
            audio_data.extend(b'fmt ')
            audio_data.extend(struct.pack('<I', 16))  # Subchunk1 size
            audio_data.extend(struct.pack('<H', 1))   # Audio format (PCM)
            audio_data.extend(struct.pack('<H', 1))   # Channels
            audio_data.extend(struct.pack('<I', sample_rate))  # Sample rate
            audio_data.extend(struct.pack('<I', sample_rate * 2))  # Byte rate
            audio_data.extend(struct.pack('<H', 2))   # Block align
            audio_data.extend(struct.pack('<H', 16))  # Bits per sample
            
            audio_data.extend(b'data')
            audio_data.extend(struct.pack('<I', num_samples * 2))
            
            # Add silent samples
            for _ in range(num_samples):
                audio_data.extend(struct.pack('<h', 0))
            
            # Save simulated audio
            filepath = self.audio_manager.save_audio(audio_data, text, voice)
            
            if filepath:
                self.get_logger().info(f'Simulated audio: {filepath}')
                self._publish_status('done', filepath)
            else:
                self._publish_status('error', 'Failed to save simulated audio')
        
        except Exception as e:
            self.get_logger().error(f'Simulated audio error: {str(e)}')
            self._publish_status('error', str(e))
    
    def _health_check(self):
        """Periodic health check of Piper server."""
        try:
            if self.sim_mode:
                self.piper_healthy = True
                return
            
            self.piper_healthy = self.piper_client.health_check()
            
            if not self.piper_healthy:
                self.get_logger().warning('Piper server health check failed')
        
        except Exception as e:
            self.get_logger().error(f'Health check error: {str(e)}')
            self.piper_healthy = False
    
    def _publish_diagnostics(self):
        """Publish system diagnostics."""
        try:
            disk_usage = self.audio_manager.get_disk_usage()
            
            diagnostics = {
                'timestamp': datetime.now().isoformat(),
                'piper_healthy': self.piper_healthy,
                'synthesis_count': self.synthesis_count,
                'error_count': self.error_count,
                'queue_size': len(self.synthesis_queue),
                'disk_usage_mb': disk_usage['total_mb'],
                'audio_file_count': disk_usage['file_count'],
            }
            
            msg = String()
            import json
            msg.data = json.dumps(diagnostics, indent=2)
            self.diagnostics_pub.publish(msg)
            
            self.get_logger().debug(
                f'Diagnostics: {self.synthesis_count} synthesized, '
                f'{self.error_count} errors, disk: {disk_usage["total_mb"]} MB'
            )
        
        except Exception as e:
            self.get_logger().error(f'Diagnostics publish error: {str(e)}')
    
    def _publish_status(self, status_type: str, message: str):
        """
        Publish TTS status message.
        
        Args:
            status_type: 'done' or 'error'
            message: Status message or filepath
        """
        try:
            msg = String()
            msg.data = f'{status_type}:{message}'
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Status publish error: {str(e)}')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.running = False
        
        # Wait for worker thread
        self.worker_thread.join(timeout=5.0)
        
        # Close Piper client
        if self.piper_client:
            self.piper_client.close()
        
        # Cancel timers
        self.health_timer.cancel()
        self.diagnostics_timer.cancel()
        
        # Cleanup old audio files
        self.audio_manager.cleanup_old_files(max_age_hours=24)
        
        self.get_logger().info('TTS Bridge node shut down')
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = SentioTTSBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
