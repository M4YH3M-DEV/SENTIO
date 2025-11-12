#!/usr/bin/env python3
"""
SENTIO Demo Runner Node

Executes choreography sequences and coordinates robot behaviors.

Subscribes to: (none directly, publishes commands)
Publishes to: /tts_text (std_msgs/String)
             /behavior_cmd (std_msgs/String)
             /demo/status (std_msgs/String)

Services:
  /demo/control (Demo control requests)
  /demo/load_sequence (Load new sequence)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import sys
import os
import logging
from typing import Optional, Dict
from pathlib import Path

from .choreography_engine import ChoreographyEngine, PlaybackState
from .sequence_parser import SequenceParser
from .demo_controller import DemoController
from .logging_utils import setup_demo_logger, log_demo_session


logger = logging.getLogger(__name__)


class DemoRunnerNode(Node):
    """Main demo runner node."""
    
    def __init__(self):
        """Initialize demo runner node."""
        super().__init__('demo_runner_node')
        
        # Declare parameters
        self.declare_parameter('choreography_dir', '')
        self.declare_parameter('default_sequence', '')
        self.declare_parameter('auto_start', False)
        
        # Get parameters
        self.choreography_dir = self.get_parameter('choreography_dir').value
        self.default_sequence = self.get_parameter('default_sequence').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # Setup logging
        self.demo_logger = setup_demo_logger('demo_runner')
        
        # QoS profiles
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.tts_pub = self.create_publisher(String, '/tts_text', cmd_qos)
        self.behavior_pub = self.create_publisher(String, '/behavior_cmd', cmd_qos)
        self.status_pub = self.create_publisher(String, '/demo/status', cmd_qos)
        
        # Components
        self.choreography = ChoreographyEngine()
        self.parser = SequenceParser()
        self.controller = DemoController(self.choreography, self.parser)
        
        # Register action callbacks
        self.choreography.register_action_callback('gesture', self._on_gesture)
        self.choreography.register_action_callback('tts', self._on_tts)
        self.choreography.register_action_callback('led', self._on_led)
        
        # Statistics
        self.demo_count = 0
        self.error_count = 0
        
        # Load sequences
        self._load_sequences()
        
        # Status timer
        self.status_timer = self.create_timer(0.5, self._publish_status)
        
        # Auto-start if specified
        if self.auto_start and self.default_sequence:
            self.controller.start_demo(self.default_sequence)
        
        self.get_logger().info(
            f'Demo Runner initialized | Sequences loaded: {len(self.controller.list_available_demos())}'
        )
    
    def _load_sequences(self):
        """Load choreography sequences from directory."""
        try:
            # Determine choreography directory
            if self.choreography_dir:
                seq_dir = self.choreography_dir
            else:
                # Use package share directory
                from ament_index_python.packages import get_package_share_directory
                pkg_dir = get_package_share_directory('sentio_demo')
                seq_dir = os.path.join(pkg_dir, 'choreography')
            
            seq_path = Path(seq_dir)
            
            if not seq_path.exists():
                self.get_logger().warning(f'Choreography directory not found: {seq_dir}')
                return
            
            # Load all YAML files
            for yaml_file in seq_path.glob('*.yaml'):
                self.parser.load_sequence(str(yaml_file))
            
            self.get_logger().info(f'Loaded {len(self.controller.list_available_demos())} sequences')
        
        except Exception as e:
            self.get_logger().error(f'Failed to load sequences: {str(e)}')
    
    def _on_gesture(self, data: Dict):
        """Handle gesture action."""
        try:
            gesture = data.get('gesture', 'idle')
            description = data.get('description', '')
            
            # Create behavior command
            behavior_cmd = {
                'gesture': gesture,
                'led': data.get('led', {
                    'color': 'white',
                    'pattern': 'steady',
                    'intensity': 0.6
                }),
                'tts': {'text': ''},
                'source': 'choreography'
            }
            
            # Publish
            msg = String()
            msg.data = json.dumps(behavior_cmd)
            self.behavior_pub.publish(msg)
            
            self.demo_logger.debug(f'Gesture: {gesture}')
        
        except Exception as e:
            self.get_logger().error(f'Gesture action error: {str(e)}')
            self.error_count += 1
    
    def _on_tts(self, data: Dict):
        """Handle TTS action."""
        try:
            text = data.get('text', '')
            
            if not text:
                return
            
            # Build TTS command
            tts_cmd = f'text:{text}'
            
            if 'voice' in data:
                tts_cmd += f'|voice:{data["voice"]}'
            
            if 'emotion' in data:
                tts_cmd += f'|emotion:{data["emotion"]}'
            
            # Publish
            msg = String()
            msg.data = tts_cmd
            self.tts_pub.publish(msg)
            
            self.demo_logger.debug(f'TTS: {text}')
        
        except Exception as e:
            self.get_logger().error(f'TTS action error: {str(e)}')
            self.error_count += 1
    
    def _on_led(self, data: Dict):
        """Handle LED action."""
        try:
            # LED actions are typically part of gesture, but can be standalone
            self.demo_logger.debug(f'LED: {data.get("color", "white")}')
        
        except Exception as e:
            self.get_logger().error(f'LED action error: {str(e)}')
    
    def _publish_status(self):
        """Publish demo status."""
        try:
            status = self.controller.get_status()
            
            # Add node stats
            status['demo_count'] = self.demo_count
            status['error_count'] = self.error_count
            status['available_sequences'] = self.controller.list_available_demos()
            
            msg = String()
            msg.data = json.dumps(status, indent=2)
            self.status_pub.publish(msg)
        
        except Exception as e:
            self.get_logger().error(f'Status publish error: {str(e)}')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.controller.stop_demo()
        self.status_timer.cancel()
        
        self.get_logger().info(
            f'Demo runner shutdown | Demos run: {self.demo_count} | Errors: {self.error_count}'
        )
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = DemoRunnerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
