#!/usr/bin/env python3
"""
SENTIO Servo Bridge Node

Maps behavior commands to servo positions and sends to hardware.

Subscribes to:
  /behavior_cmd (std_msgs/String) - JSON behavior command
  /motion/override (std_msgs/String) - Emergency override
  /imu/data (sensor_msgs/Imu) - IMU data for safety

Publishes to:
  /motion/status (std_msgs/String) - JSON status
  /motion/feedback (std_msgs/String) - Servo feedback
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Imu
import json
import sys
import yaml
import logging
import argparse
from typing import Optional, Dict
from pathlib import Path

from .kinematics_engine import KinematicsEngine
from .motion_controller import MotionController, MotionState
from .safety_monitor import SafetyMonitor
from .serial_interface import SerialInterface
from .servo_command import ServoCommand, LEDCommand
from .logging_utils import setup_motion_logger, log_motion_command, log_safety_event


logger = logging.getLogger(__name__)


class ServoBridgeNode(Node):
    """Main servo bridge node."""
    
    def __init__(self, simulate: bool = False):
        """
        Initialize servo bridge node.
        
        Args:
            simulate: Run in simulation mode without hardware
        """
        super().__init__('servo_bridge_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('kinematics_config', '')
        self.declare_parameter('servo_map_config', '')
        self.declare_parameter('safety_limits_config', '')
        self.declare_parameter('simulate_hardware', False)
        self.declare_parameter('max_velocity_dps', 60.0)
        self.declare_parameter('enable_smoothing', True)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.kinematics_config = self.get_parameter('kinematics_config').value
        self.servo_map_config = self.get_parameter('servo_map_config').value
        self.safety_limits_config = self.get_parameter('safety_limits_config').value
        self.simulate_hardware = self.get_parameter('simulate_hardware').value or simulate
        self.max_velocity_dps = self.get_parameter('max_velocity_dps').value
        self.enable_smoothing = self.get_parameter('enable_smoothing').value
        
        # QoS profiles
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/motion/status', command_qos)
        self.feedback_pub = self.create_publisher(String, '/motion/feedback', command_qos)
        
        # Subscribers
        self.behavior_sub = self.create_subscription(
            String,
            '/behavior_cmd',
            self.behavior_callback,
            command_qos
        )
        
        self.override_sub = self.create_subscription(
            String,
            '/motion/override',
            self.override_callback,
            command_qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            imu_qos
        )
        
        # Components
        self.kinematics = KinematicsEngine(self.kinematics_config)
        self.motion_controller = MotionController()
        self.safety_monitor = SafetyMonitor(max_velocity_dps=self.max_velocity_dps)
        
        # Hardware interface
        self.serial_interface: Optional[SerialInterface] = None
        if not self.simulate_hardware:
            self.serial_interface = SerialInterface(
                port=self.serial_port,
                baudrate=self.baudrate
            )
            self._initialize_hardware()
        
        # Statistics
        self.command_count = 0
        self.error_count = 0
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self._publish_status)
        
        self.get_logger().info(
            f'Servo Bridge initialized | Mode: {"SIMULATION" if self.simulate_hardware else "HARDWARE"} | '
            f'Smoothing: {self.enable_smoothing} | Safety: enabled'
        )
    
    def _initialize_hardware(self):
        """Initialize hardware connection."""
        if not self.serial_interface:
            return
        
        if self.serial_interface.connect():
            self.get_logger().info('Hardware initialized successfully')
            
            # Send initialization command
            init_cmd = {'type': 'init', 'servo_count': 5}
            success, response = self.serial_interface.send_command(init_cmd)
            
            if success:
                self.get_logger().info(f'Initialization response: {response}')
            else:
                self.get_logger().warning('Initialization failed, continuing anyway')
        else:
            self.get_logger().error('Failed to connect to hardware')
    
    def behavior_callback(self, msg: String):
        """Handle behavior command."""
        try:
            behavior_cmd = json.loads(msg.data)
            self._execute_behavior(behavior_cmd)
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid behavior JSON: {str(e)}')
            self.error_count += 1
    
    def override_callback(self, msg: String):
        """Handle emergency override."""
        try:
            override_cmd = json.loads(msg.data)
            
            if override_cmd.get('type') == 'estop':
                self.safety_monitor.activate_emergency_stop('User requested E-stop')
                self._send_safe_shutdown()
            elif override_cmd.get('type') == 'clear_estop':
                self.safety_monitor.deactivate_emergency_stop()
        
        except Exception as e:
            self.get_logger().error(f'Override error: {str(e)}')
    
    def imu_callback(self, msg: Imu):
        """Handle IMU data."""
        self.safety_monitor.update_imu(msg)
        
        # Check for fall
        is_falling, reason = self.safety_monitor.check_fall()
        if is_falling:
            self.get_logger().warning(f'Fall detected: {reason}')
            self.safety_monitor.activate_emergency_stop(reason)
            self._send_safe_shutdown()
    
    def _execute_behavior(self, behavior_cmd: Dict):
        """Execute a behavior command."""
        try:
            if self.safety_monitor.check_emergency_stop():
                self.get_logger().warning('Motion blocked by emergency stop')
                return
            
            gesture = behavior_cmd.get('gesture', 'idle')
            gesture_def = self.kinematics.get_gesture(gesture)
            
            if not gesture_def:
                self.get_logger().warning(f'Unknown gesture: {gesture}')
                gesture_def = self.kinematics.get_gesture('idle')
            
            # Extract servo positions
            if 'positions' in gesture_def:
                positions = gesture_def['positions']
            else:
                positions = self.kinematics.get_neutral_pose()
            
            # Clamp angles to servo limits
            clamped_positions = {
                servo_id: self.kinematics.clamp_angle(servo_id, angle)
                for servo_id, angle in positions.items()
            }
            
            # Create servo command
            servo_cmd = ServoCommand()
            servo_cmd.positions = clamped_positions
            servo_cmd.duration_ms = gesture_def.get('duration_ms', 500)
            servo_cmd.velocity_limit = self.max_velocity_dps
            servo_cmd.smoothing_enabled = self.enable_smoothing
            
            # Send command
            self._send_servo_command(servo_cmd)
            
            # Handle LED from behavior
            if 'led' in behavior_cmd:
                led_cmd = LEDCommand(
                    color=self._parse_color(behavior_cmd['led'].get('color', 'white')),
                    pattern=behavior_cmd['led'].get('pattern', 'solid'),
                    intensity=behavior_cmd['led'].get('intensity', 0.8)
                )
                self._send_led_command(led_cmd)
            
            self.command_count += 1
            log_motion_command(self.get_logger(), servo_cmd.positions, gesture)
        
        except Exception as e:
            self.get_logger().error(f'Behavior execution error: {str(e)}')
            self.error_count += 1
    
    def _send_servo_command(self, servo_cmd: ServoCommand):
        """Send servo command to hardware."""
        cmd_json = json.loads(servo_cmd.to_json())
        
        if self.simulate_hardware:
            self.get_logger().debug(f'[SIM] Servo command: {cmd_json}')
        else:
            if self.serial_interface and self.serial_interface.is_connected():
                success, response = self.serial_interface.send_command(cmd_json)
                
                if success:
                    self.get_logger().debug(f'Servo command sent successfully')
                else:
                    self.get_logger().error(f'Servo command failed: {response}')
                    self.error_count += 1
    
    def _send_led_command(self, led_cmd: LEDCommand):
        """Send LED command to hardware."""
        cmd_json = json.loads(led_cmd.to_json())
        
        if self.simulate_hardware:
            self.get_logger().debug(f'[SIM] LED command: {cmd_json}')
        else:
            if self.serial_interface and self.serial_interface.is_connected():
                success, _ = self.serial_interface.send_command(cmd_json)
                
                if not success:
                    self.get_logger().warning('LED command failed')
    
    def _send_safe_shutdown(self):
        """Send safe shutdown command to servos."""
        neutral = self.kinematics.get_neutral_pose()
        servo_cmd = ServoCommand()
        servo_cmd.positions = neutral
        servo_cmd.duration_ms = 1000
        
        self._send_servo_command(servo_cmd)
    
    def _parse_color(self, color_str: str) -> tuple:
        """Parse color string to RGB tuple."""
        color_map = {
            'white': (255, 255, 255),
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'blue': (0, 0, 255),
            'yellow': (255, 255, 0),
            'cyan': (0, 255, 255),
            'magenta': (255, 0, 255),
            'orange': (255, 165, 0),
            'off': (0, 0, 0),
        }
        return color_map.get(color_str.lower(), (255, 255, 255))
    
    def _publish_status(self):
        """Publish motion status."""
        try:
            status = {
                'motion_state': self.motion_controller.state.value,
                'commands_sent': self.command_count,
                'errors': self.error_count,
                'safety_status': self.safety_monitor.get_safety_status(),
                'hardware_connected': (
                    self.serial_interface.is_connected() 
                    if self.serial_interface else False
                ),
                'simulate_mode': self.simulate_hardware
            }
            
            msg = String()
            msg.data = json.dumps(status, indent=2)
            self.status_pub.publish(msg)
        
        except Exception as e:
            self.get_logger().error(f'Status publish error: {str(e)}')
    
    def destroy_node(self):
        """Clean shutdown."""
        if self.serial_interface:
            self.serial_interface.disconnect()
        
        self.status_timer.cancel()
        self.get_logger().info(
            f'Servo bridge shutdown | Commands: {self.command_count} | Errors: {self.error_count}'
        )
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    parser = argparse.ArgumentParser(description='SENTIO Servo Bridge')
    parser.add_argument('--simulate', action='store_true', help='Run in simulation mode')
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    try:
        node = ServoBridgeNode(simulate=parsed_args.simulate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
