#!/usr/bin/env python3
"""
Example: Trigger demo via rosbridge (HTTP REST API)

Prerequisites:
  ros2 launch rosbridge_server rosbridge_websocket.launch.xml

Usage:
  python rosbridge_trigger_demo.py --sequence greet_sequence --visitor Alice
"""

import requests
import json
import argparse
import time


def trigger_demo_via_rosservice(rosbridge_url: str, sequence_name: str, variables=None):
    """
    Trigger demo via ROS service call over rosbridge.
    
    Args:
        rosbridge_url: URL of rosbridge (e.g., http://localhost:9090)
        sequence_name: Name of sequence to run
        variables: Optional dict of variables
    """
    # ROS service call via HTTP REST
    service_url = f'{rosbridge_url}/call_service'
    
    request_body = {
        'service': '/demo/start',
        'args': {
            'sequence_name': sequence_name,
            'variables': variables or {}
        }
    }
    
    print(f'Triggering demo: {sequence_name}')
    print(f'Service URL: {service_url}')
    print(f'Request: {json.dumps(request_body, indent=2)}')
    
    try:
        response = requests.post(
            service_url,
            json=request_body,
            timeout=5.0
        )
        
        if response.status_code == 200:
            result = response.json()
            print(f'Response: {json.dumps(result, indent=2)}')
        else:
            print(f'Error: HTTP {response.status_code}')
            print(response.text)
    
    except requests.RequestException as e:
        print(f'Request error: {str(e)}')


def monitor_demo_status(rosbridge_url: str):
    """
    Monitor demo status via rosbridge topic.
    
    Args:
        rosbridge_url: URL of rosbridge
    """
    subscribe_url = f'{rosbridge_url}/subscribe'
    
    # Subscribe to /demo/status
    request_body = {
        'topic': '/demo/status',
        'type': 'std_msgs/String',
        'throttle_rate': 1000,
        'queue_length': 1
    }
    
    print(f'Subscribing to /demo/status')
    print(f'Subscribe URL: {subscribe_url}')
    
    # In a real implementation, would use websocket to receive updates
    print('(Note: Full websocket implementation would be used in production)')


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Trigger SENTIO demo via rosbridge')
    parser.add_argument('--rosbridge-url', default='http://localhost:9090',
                       help='rosbridge URL')
    parser.add_argument('--sequence', default='greet_sequence',
                       help='Sequence to run')
    parser.add_argument('--visitor', help='Visitor name (optional)')
    
    args = parser.parse_args()
    
    # Prepare variables
    variables = {}
    if args.visitor:
        variables['visitor_name'] = args.visitor
    
    # Trigger demo
    trigger_demo_via_rosservice(args.rosbridge_url, args.sequence, variables)
    
    # Monitor status
    print('\nMonitoring demo...')
    for i in range(5):
        time.sleep(1)
        print(f'[{i+1}s] Waiting...')


if __name__ == '__main__':
    main()
