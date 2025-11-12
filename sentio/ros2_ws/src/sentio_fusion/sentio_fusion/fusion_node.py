#!/usr/bin/env python3
"""
AETHER Fusion Node

Multimodal affect fusion combining vision and audio emotions.

Subscribes to: /affect/vision (sentio_msgs/Affect)
              /affect/audio (sentio_msgs/Affect)
Publishes to: /affect (sentio_msgs/Affect) - unified affect
             /aether/reasoning (std_msgs/String) - reasoning explanation
             /aether/diagnostics (std_msgs/String) - diagnostics/stats
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import sys
import logging
import json
import threading
from typing import Optional
from datetime import datetime

from sentio_msgs.msg import Affect

# Import AETHER fusion (adjust path as needed)
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent.parent))

from aether.fusion import FusionEngine
from aether.explainability import ReasoningTrail, DecisionLogger, ExplanationFormatter


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FusionNode(Node):
    """AETHER multimodal fusion node."""
    
    def __init__(self):
        """Initialize fusion node."""
        super().__init__('fusion_node')
        
        # Declare parameters
        self.declare_parameter('fusion_strategy', 'confidence_weighted')
        self.declare_parameter('fusion_config_file', '')
        self.declare_parameter('log_dir', '/root/aether_sentio_ws/logs')
        self.declare_parameter('record_reasoning', True)
        self.declare_parameter('smoothing_enabled', True)
        self.declare_parameter('sim_mode', False)
        
        # Get parameters
        self.fusion_strategy = self.get_parameter('fusion_strategy').value
        self.fusion_config = self.get_parameter('fusion_config_file').value
        self.log_dir = self.get_parameter('log_dir').value
        self.record_reasoning = self.get_parameter('record_reasoning').value
        self.smoothing_enabled = self.get_parameter('smoothing_enabled').value
        self.sim_mode = self.get_parameter('sim_mode').value
        
        # QoS profiles
        affect_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.unified_affect_pub = self.create_publisher(
            Affect,
            '/affect',
            affect_qos
        )
        
        self.reasoning_pub = self.create_publisher(
            String,
            '/aether/reasoning',
            affect_qos
        )
        
        self.diagnostics_pub = self.create_publisher(
            String,
            '/aether/diagnostics',
            affect_qos
        )
        
        # Subscribers
        self.vision_sub = self.create_subscription(
            Affect,
            '/affect/vision',
            self.vision_callback,
            affect_qos
        )
        
        self.audio_sub = self.create_subscription(
            Affect,
            '/affect/audio',
            self.audio_callback,
            affect_qos
        )
        
        # AETHER components
        self.fusion_engine = FusionEngine(history_size=30)
        self.reasoning_trail = ReasoningTrail(self.log_dir)
        self.decision_logger = DecisionLogger(self.log_dir)
        
        # State tracking
        self.last_vision_affect: Optional[Affect] = None
        self.last_audio_affect: Optional[Affect] = None
        self.lock = threading.Lock()
        self.fusion_count = 0
        
        # Diagnostics timer
        self.diagnostics_timer = self.create_timer(10.0, self._publish_diagnostics)
        
        self.get_logger().info(
            f'Fusion Node initialized | Strategy: {self.fusion_strategy} | '
            f'Smoothing: {self.smoothing_enabled} | SIM: {self.sim_mode}'
        )
    
    def vision_callback(self, msg: Affect):
        """Handle vision affect update."""
        with self.lock:
            self.last_vision_affect = msg
        
        self._attempt_fusion()
    
    def audio_callback(self, msg: Affect):
        """Handle audio affect update."""
        with self.lock:
            self.last_audio_affect = msg
        
        self._attempt_fusion()
    
    def _attempt_fusion(self):
        """Attempt fusion if both modalities available."""
        with self.lock:
            if self.last_vision_affect is None or self.last_audio_affect is None:
                return
            
            vision_msg = self.last_vision_affect
            audio_msg = self.last_audio_affect
        
        # Convert ROS messages to dicts
        vision_affect = {
            'valence': float(vision_msg.valence),
            'arousal': float(vision_msg.arousal),
            'confidence': 0.8  # Default, could extract from label_text
        }
        
        audio_affect = {
            'valence': float(audio_msg.valence),
            'arousal': float(audio_msg.arousal),
            'confidence': 0.7
        }
        
        # Start reasoning trail
        if self.record_reasoning:
            self.reasoning_trail.start_decision_chain('affect_fusion')
            
            self.reasoning_trail.add_entry(
                'vision_input',
                {'valence': vision_msg.valence, 'arousal': vision_msg.arousal},
                confidence=0.8,
                reasoning='Facial expression detected'
            )
            
            self.reasoning_trail.add_entry(
                'audio_input',
                {'valence': audio_msg.valence, 'arousal': audio_msg.arousal},
                confidence=0.7,
                reasoning='Speech tone analyzed'
            )
        
        # Perform fusion
        fused = self.fusion_engine.process_inputs(
            vision_affect,
            audio_affect,
            use_history=self.smoothing_enabled
        )
        
        if not fused:
            return
        
        # Log decision
        self.decision_logger.log_decision(
            decision_id=f'fusion_{self.fusion_count}',
            decision_type='multimodal_affect_fusion',
            inputs={'vision': vision_affect, 'audio': audio_affect},
            outputs=fused,
            metadata={
                'strategy': self.fusion_strategy,
                'consistency': fused.get('consistency_score', 0),
                'sim_mode': self.sim_mode
            }
        )
        
        # End reasoning trail
        if self.record_reasoning:
            self.reasoning_trail.add_entry(
                'fusion',
                fused,
                confidence=fused.get('confidence', 0.5),
                reasoning=f"Fused affect with VA({fused['valence']:.2f}, {fused['arousal']:.2f})"
            )
            
            trail_id = self.reasoning_trail.end_decision_chain(
                decision=fused,
                outcome='fusion_successful'
            )
            fused['reasoning_trail_id'] = trail_id
        
        # Publish unified affect
        self._publish_unified_affect(fused)
        
        # Publish reasoning
        self._publish_reasoning(fused)
        
        self.fusion_count += 1
    
    def _publish_unified_affect(self, fused: dict):
        """Publish unified affect message."""
        try:
            msg = Affect()
            msg.valence = float(fused.get('valence', 0.0))
            msg.arousal = float(fused.get('arousal', 0.3))
            msg.label = int(self._va_to_label(msg.valence, msg.arousal))
            msg.label_text = f"fusion:VA({msg.valence:.2f},{msg.arousal:.2f})"
            
            self.unified_affect_pub.publish(msg)
            
            self.get_logger().debug(
                f'Unified affect: VA({msg.valence:.2f}, {msg.arousal:.2f})'
            )
        
        except Exception as e:
            self.get_logger().error(f'Unified affect publish error: {str(e)}')
    
    def _publish_reasoning(self, fused: dict):
        """Publish reasoning explanation."""
        try:
            reasoning = fused.get('reasoning', {})
            
            explanation = ExplanationFormatter.format_fusion_explanation(
                reasoning=reasoning,
                decision=fused
            )
            
            msg = String()
            msg.data = explanation
            self.reasoning_pub.publish(msg)
        
        except Exception as e:
            self.get_logger().error(f'Reasoning publish error: {str(e)}')
    
    def _publish_diagnostics(self):
        """Publish diagnostics information."""
        try:
            trend = self.fusion_engine.get_trend()
            anomalies = self.fusion_engine.detect_anomalies()
            stats = self.decision_logger.get_decision_stats()
            
            diagnostics = {
                'timestamp': datetime.now().isoformat(),
                'fusion_count': self.fusion_count,
                'consistency_score': self.fusion_engine.consistency_score,
                'trend': trend,
                'anomalies_detected': anomalies.get('is_anomalous', False),
                'decision_stats': stats
            }
            
            msg = String()
            msg.data = json.dumps(diagnostics, indent=2)
            self.diagnostics_pub.publish(msg)
        
        except Exception as e:
            self.get_logger().error(f'Diagnostics publish error: {str(e)}')
    
    def _va_to_label(self, valence: float, arousal: float) -> int:
        """Convert valence-arousal to emotion label."""
        # Simplified mapping
        if valence > 0.5 and arousal > 0.5:
            return 1  # Happy
        elif valence < -0.5 and arousal < 0.5:
            return 2  # Sad
        elif valence < -0.5 and arousal > 0.5:
            return 3  # Angry
        else:
            return 0  # Neutral
    
    def destroy_node(self):
        """Clean shutdown."""
        self.diagnostics_timer.cancel()
        self.get_logger().info(f'Fusion node shutdown | Total fusions: {self.fusion_count}')
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = FusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
