#!/usr/bin/env python3
"""
SENTIO Policy Engine Node

Maps unified affect to robot behaviors and voice commands.

Subscribes to:
  /affect (sentio_msgs/Affect) - unified emotion state
  /tfmini/distance (sensor_msgs/Range) - proximity
  /cameraB/group_count (std_msgs/Int32) - group size
  /cameraB/faces (std_msgs/String) - face detections

Publishes to:
  /behavior_cmd (std_msgs/String) - behavior JSON
  /tts_text (std_msgs/String) - text for TTS
  /policy/status (std_msgs/String) - diagnostics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Range
import json
import logging
import sys
import yaml
import threading
from typing import Dict, Optional
from pathlib import Path

from sentio_msgs.msg import Affect
from .rule_evaluator import RuleEvaluator
from .behavior_mapper import BehaviorMapper
from .validators import BehaviorValidator
from .logging_utils import log_policy_decision


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class PolicyEngineNode(Node):
    """SENTIO Policy Engine node."""
    
    def __init__(self):
        """Initialize policy engine node."""
        super().__init__('policy_engine_node')
        
        # Declare parameters
        self.declare_parameter('policy_config_file', '')
        self.declare_parameter('schema_file', '')
        self.declare_parameter('default_profile', 'balanced')
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('safety_enabled', True)
        
        # Get parameters
        policy_config = self.get_parameter('policy_config_file').value
        schema_file = self.get_parameter('schema_file').value
        self.default_profile = self.get_parameter('default_profile').value
        self.sim_mode = self.get_parameter('sim_mode').value
        self.safety_enabled = self.get_parameter('safety_enabled').value
        
        # QoS profiles
        affect_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        behavior_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.behavior_pub = self.create_publisher(
            String,
            '/behavior_cmd',
            behavior_qos
        )
        
        self.tts_pub = self.create_publisher(
            String,
            '/tts_text',
            behavior_qos
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/policy/status',
            behavior_qos
        )
        
        # Subscribers
        self.affect_sub = self.create_subscription(
            Affect,
            '/affect',
            self.affect_callback,
            affect_qos
        )
        
        self.proximity_sub = self.create_subscription(
            Range,
            '/tfmini/distance',
            self.proximity_callback,
            sensor_qos
        )
        
        self.group_count_sub = self.create_subscription(
            Int32,
            '/cameraB/group_count',
            self.group_count_callback,
            sensor_qos
        )
        
        self.faces_sub = self.create_subscription(
            String,
            '/cameraB/faces',
            self.faces_callback,
            sensor_qos
        )
        
        # Components
        self.rule_evaluator = RuleEvaluator()
        self.behavior_mapper = BehaviorMapper()
        self.validator = BehaviorValidator(schema_file)
        
        # State tracking
        self.current_affect: Optional[Dict] = None
        self.current_context: Dict = {
            'proximity_m': 10.0,
            'group_count': 0,
            'faces': []
        }
        self.lock = threading.Lock()
        
        # Load policy configuration
        self.policy_rules = self._load_policy_config(policy_config)
        
        # Statistics
        self.behavior_count = 0
        self.decision_log = []
        
        # Status timer
        self.status_timer = self.create_timer(10.0, self._publish_status)
        
        self.get_logger().info(
            f'Policy Engine initialized | Rules: {len(self.policy_rules)} | '
            f'Safety: {self.safety_enabled} | SIM: {self.sim_mode}'
        )
    
    def _load_policy_config(self, config_file: str) -> list:
        """Load policy configuration from YAML file."""
        try:
            if config_file and Path(config_file).exists():
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                    rules = config.get('rules', [])
                    logger.info(f'Loaded {len(rules)} policy rules from {config_file}')
                    return rules
            else:
                logger.warning('Policy config file not specified or not found')
                return self._get_default_rules()
        
        except Exception as e:
            logger.error(f'Failed to load policy config: {str(e)}')
            return self._get_default_rules()
    
    def _get_default_rules(self) -> list:
        """Get default policy rules."""
        return [
            {
                'name': 'happy_greet',
                'priority': 100,
                'condition': {'field': 'affect.valence', 'operator': 'gt', 'value': 0.6},
                'behavior': 'happy_greet'
            },
            {
                'name': 'sad_sympathize',
                'priority': 90,
                'condition': {'field': 'affect.valence', 'operator': 'lt', 'value': -0.6},
                'behavior': 'sad_respond'
            },
            {
                'name': 'approach_greet',
                'priority': 80,
                'conditions': [
                    {'field': 'context.group_count', 'operator': 'gt', 'value': 0},
                    {'field': 'context.proximity_m', 'operator': 'lt', 'value': 1.0}
                ],
                'behavior': 'approach_greet'
            }
        ]
    
    def affect_callback(self, msg: Affect):
        """Handle affect updates."""
        with self.lock:
            self.current_affect = {
                'valence': float(msg.valence),
                'arousal': float(msg.arousal),
                'label': int(msg.label),
                'label_text': msg.label_text
            }
        
        self._process_affect()
    
    def proximity_callback(self, msg: Range):
        """Handle proximity updates."""
        with self.lock:
            self.current_context['proximity_m'] = float(msg.range)
    
    def group_count_callback(self, msg: Int32):
        """Handle group count updates."""
        with self.lock:
            self.current_context['group_count'] = int(msg.data)
    
    def faces_callback(self, msg: String):
        """Handle face detection updates."""
        try:
            faces_data = json.loads(msg.data)
            with self.lock:
                self.current_context['faces'] = faces_data.get('detections', [])
        except json.JSONDecodeError:
            pass
    
    def _process_affect(self):
        """Process affect and make policy decision."""
        try:
            with self.lock:
                if not self.current_affect:
                    return
                
                affect = dict(self.current_affect)
                context = dict(self.current_context)
            
            # Prepare evaluation context
            eval_context = {
                'affect': affect,
                'context': context
            }
            
            # Check context-based behaviors first
            context_behavior = self.behavior_mapper.map_context_to_behavior(context)
            if context_behavior:
                self._publish_behavior(context_behavior)
                return
            
            # Evaluate policy rules
            matched_rules = self.rule_evaluator.evaluate_rules(
                self.policy_rules,
                eval_context,
                match_first=False
            )
            
            # Select highest priority matched rule
            if matched_rules:
                best_rule = max(matched_rules, key=lambda r: r['priority'])
                
                # Map to behavior
                emotion = self._label_to_emotion(affect.get('label', 0))
                behavior = self.behavior_mapper.map_emotion_to_behavior(
                    emotion,
                    affect.get('valence', 0),
                    affect.get('arousal', 0.3),
                    confidence=1.0,
                    context=context
                )
            else:
                # Default to neutral/idle
                behavior = self.behavior_mapper.create_idle_behavior()
            
            # Publish behavior
            self._publish_behavior(behavior)
            
            # Log decision
            log_policy_decision(
                logger,
                'affect_to_behavior',
                inputs=affect,
                decision=behavior,
                confidence=affect.get('confidence', 0.5)
            )
        
        except Exception as e:
            logger.error(f'Affect processing error: {str(e)}')
            error_behavior = self.behavior_mapper.create_error_behavior(str(e))
            self._publish_behavior(error_behavior)
    
    def _publish_behavior(self, behavior: Dict):
        """Publish behavior command."""
        try:
            # Validate behavior
            is_valid, error_msg = self.validator.validate(behavior)
            
            if not is_valid:
                logger.error(f'Invalid behavior: {error_msg}')
                if self.safety_enabled:
                    behavior = self.behavior_mapper.create_idle_behavior()
                else:
                    return
            
            # Publish behavior command
            behavior_msg = String()
            behavior_msg.data = json.dumps(behavior)
            self.behavior_pub.publish(behavior_msg)
            
            # Publish TTS if present
            if 'tts' in behavior and behavior['tts'].get('text'):
                tts_msg = String()
                tts_text = behavior['tts']['text']
                if 'voice' in behavior['tts']:
                    tts_text += f"|voice:{behavior['tts']['voice']}"
                if 'emotion' in behavior['tts']:
                    tts_text += f"|emotion:{behavior['tts']['emotion']}"
                
                tts_msg.data = tts_text
                self.tts_pub.publish(tts_msg)
            
            self.behavior_count += 1
            
            logger.debug(f'Published behavior: {behavior.get("gesture", "unknown")}')
        
        except Exception as e:
            logger.error(f'Behavior publish error: {str(e)}')
    
    def _publish_status(self):
        """Publish diagnostics status."""
        try:
            status = {
                'behavior_count': self.behavior_count,
                'rules_loaded': len(self.policy_rules),
                'current_context': self.current_context,
                'safety_enabled': self.safety_enabled,
                'sim_mode': self.sim_mode
            }
            
            msg = String()
            msg.data = json.dumps(status, indent=2)
            self.status_pub.publish(msg)
        
        except Exception as e:
            logger.error(f'Status publish error: {str(e)}')
    
    def _label_to_emotion(self, label: int) -> str:
        """Convert emotion label to name."""
        label_map = {
            0: 'neutral',
            1: 'happy',
            2: 'sad',
            3: 'angry',
            4: 'fearful',
            5: 'disgusted',
            6: 'surprised'
        }
        return label_map.get(label, 'neutral')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.status_timer.cancel()
        self.get_logger().info(f'Policy engine shutdown | Total behaviors: {self.behavior_count}')
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = PolicyEngineNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
