#!/usr/bin/env python3
"""
SENTIO Core Node
================

Main system manager for SENTIO. Provides:
- Heartbeat monitoring (/sentio/heartbeat)
- Error logging and diagnostics (/sentio/errors)
- System health checks
- Node lifecycle coordination

This is the foundational node that should start first in any SENTIO launch.

Author: DevSora Deep-Tech Research
License: Proprietary
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String
import sys
import signal
import traceback
from datetime import datetime
import json


class SentioCoreNode(Node):
    """
    SENTIO Core system manager node.
    
    Responsibilities:
    - Publish periodic heartbeat signal
    - Listen for error messages from subsystems
    - Maintain system health status
    - Provide diagnostics information
    """
    
    def __init__(self):
        """Initialize the SENTIO Core node."""
        super().__init__('sentio_core_node')
        
        # Declare parameters
        self.declare_parameter('heartbeat_rate_hz', 1.0)
        self.declare_parameter('log_level', 'INFO')
        self.declare_parameter('enable_diagnostics', True)
        
        # Get parameters
        self.heartbeat_rate = self.get_parameter('heartbeat_rate_hz').value
        self.log_level = self.get_parameter('log_level').value
        self.enable_diagnostics = self.get_parameter('enable_diagnostics').value
        
        # QoS profiles
        # Heartbeat uses transient local for late joiners to see system is alive
        heartbeat_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Error messages use reliable delivery
        error_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        
        # Publishers
        self.heartbeat_pub = self.create_publisher(
            Bool,
            '/sentio/heartbeat',
            heartbeat_qos
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/sentio/status',
            error_qos
        )
        
        # Subscribers
        self.error_sub = self.create_subscription(
            String,
            '/sentio/errors',
            self.error_callback,
            error_qos
        )
        
        # State tracking
        self.is_healthy = True
        self.error_count = 0
        self.start_time = self.get_clock().now()
        self.last_heartbeat = None
        self.error_log = []
        
        # Heartbeat timer
        self.heartbeat_timer = self.create_timer(
            1.0 / self.heartbeat_rate,
            self.publish_heartbeat
        )
        
        # Status timer (every 10 seconds)
        if self.enable_diagnostics:
            self.status_timer = self.create_timer(
                10.0,
                self.publish_status
            )
        
        self.get_logger().info(
            f'SENTIO Core Node initialized | Heartbeat: {self.heartbeat_rate} Hz'
        )
        self.get_logger().info('System startup complete. Beginning heartbeat.')
        
        # Publish initial status
        self.publish_status()
    
    def publish_heartbeat(self):
        """Publish heartbeat signal to indicate system is alive."""
        try:
            msg = Bool()
            msg.data = self.is_healthy
            self.heartbeat_pub.publish(msg)
            self.last_heartbeat = self.get_clock().now()
            
            # Log at debug level to avoid spam
            if self.log_level == 'DEBUG':
                self.get_logger().debug('Heartbeat published')
                
        except Exception as e:
            self.get_logger().error(f'Failed to publish heartbeat: {str(e)}')
            self.is_healthy = False
    
    def error_callback(self, msg: String):
        """
        Handle error messages from other SENTIO subsystems.
        
        Args:
            msg: Error message string
        """
        error_data = {
            'timestamp': datetime.now().isoformat(),
            'message': msg.data,
            'error_count': self.error_count + 1
        }
        
        self.error_log.append(error_data)
        self.error_count += 1
        
        # Keep only last 100 errors in memory
        if len(self.error_log) > 100:
            self.error_log.pop(0)
        
        self.get_logger().warn(f'Error received: {msg.data}')
        
        # If too many errors, mark system as unhealthy
        if self.error_count > 50:
            self.is_healthy = False
            self.get_logger().error(
                'System marked unhealthy due to excessive errors'
            )
    
    def publish_status(self):
        """Publish detailed system status information."""
        try:
            uptime = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            
            status_data = {
                'node': 'sentio_core',
                'healthy': self.is_healthy,
                'uptime_seconds': round(uptime, 2),
                'error_count': self.error_count,
                'heartbeat_rate_hz': self.heartbeat_rate,
                'timestamp': datetime.now().isoformat()
            }
            
            msg = String()
            msg.data = json.dumps(status_data, indent=2)
            self.status_pub.publish(msg)
            
            self.get_logger().info(
                f'Status: {"HEALTHY" if self.is_healthy else "UNHEALTHY"} | '
                f'Uptime: {uptime:.1f}s | Errors: {self.error_count}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish status: {str(e)}')
    
    def shutdown(self):
        """Clean shutdown of the core node."""
        self.get_logger().info('SENTIO Core shutting down...')
        
        # Publish final status
        self.is_healthy = False
        self.publish_status()
        
        # Destroy timers
        if hasattr(self, 'heartbeat_timer'):
            self.heartbeat_timer.cancel()
        if hasattr(self, 'status_timer'):
            self.status_timer.cancel()
        
        self.get_logger().info('SENTIO Core shutdown complete.')


def signal_handler(sig, frame, node):
    """Handle shutdown signals gracefully."""
    print('\nShutdown signal received. Cleaning up...')
    if node:
        node.shutdown()
    sys.exit(0)


def main(args=None):
    """Main entry point for sentio_core_node."""
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create node
    node = None
    
    try:
        node = SentioCoreNode()
        
        # Register signal handlers
        signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, node))
        signal.signal(signal.SIGTERM, lambda s, f: signal_handler(s, f, node))
        
        # Create executor and spin
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            node.shutdown()
            
    except Exception as e:
        print(f'Fatal error in sentio_core_node: {str(e)}')
        traceback.print_exc()
        return 1
    
    finally:
        # Cleanup
        if node:
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
