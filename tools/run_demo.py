#!/usr/bin/env python3
"""
CLI tool to run SENTIO demo sequences locally.

Usage:
    python run_demo.py --sequence greet_sequence --simulate
    python run_demo.py --sequence showcase_full --visitor Alice
    python run_demo.py --list
"""

import sys
import argparse
import logging
from pathlib import Path

# Add ROS2 workspace to path
ros_ws = Path.home() / 'Documents/DEVSORA PROJECTS/SENTIO/sentio/ros2_ws'
if ros_ws.exists():
    sys.path.insert(0, str(ros_ws / 'install/sentio_demo/lib/python3.10/site-packages'))

from sentio_demo.sequence_parser import SequenceParser
from sentio_demo.choreography_engine import ChoreographyEngine
from sentio_demo.demo_controller import DemoController


# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def find_choreography_dir():
    """Find choreography directory."""
    # Try ROS package share directory
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_dir = get_package_share_directory('sentio_demo')
        choro_dir = Path(pkg_dir) / 'choreography'
        if choro_dir.exists():
            return choro_dir
    except:
        pass
    
    # Try relative to script
    script_dir = Path(__file__).parent.parent / 'sentio/ros2_ws/src/sentio_demo/choreography'
    if script_dir.exists():
        return script_dir
    
    return None


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='SENTIO Demo CLI')
    parser.add_argument('--sequence', help='Sequence to run')
    parser.add_argument('--list', action='store_true', help='List available sequences')
    parser.add_argument('--visitor', help='Visitor name (optional)')
    parser.add_argument('--simulate', action='store_true', help='Simulate without publishing')
    
    args = parser.parse_args()
    
    # Find choreography directory
    choro_dir = find_choreography_dir()
    if not choro_dir:
        logger.error('Choreography directory not found')
        return 1
    
    logger.info(f'Using choreography directory: {choro_dir}')
    
    # Initialize components
    parser_obj = SequenceParser()
    
    # Load sequences
    for yaml_file in choro_dir.glob('*.yaml'):
        parser_obj.load_sequence(str(yaml_file))
    
    logger.info(f'Loaded {len(parser_obj.list_sequences())} sequences')
    
    # List sequences
    if args.list:
        print('\nAvailable sequences:')
        for seq_name in parser_obj.list_sequences():
            seq = parser_obj.get_sequence(seq_name)
            print(f'  - {seq_name}: {seq.get("description", "")}')
        return 0
    
    # Run sequence
    if args.sequence:
        if args.sequence not in parser_obj.list_sequences():
            logger.error(f'Sequence not found: {args.sequence}')
            return 1
        
        # Prepare variables
        variables = {}
        if args.visitor:
            variables['visitor_name'] = args.visitor
        
        # Create mock controllers if not simulating
        if not args.simulate:
            logger.info('Would publish to ROS topics (use --simulate to test locally)')
        
        # Execute sequence locally
        choreography = ChoreographyEngine()
        
        # Mock callbacks
        def mock_gesture(data):
            gesture = data.get('gesture', 'unknown')
            logger.info(f'[GESTURE] {gesture}')
        
        def mock_tts(data):
            text = data.get('text', '')
            logger.info(f'[TTS] {text}')
        
        choreography.register_action_callback('gesture', mock_gesture)
        choreography.register_action_callback('tts', mock_tts)
        
        # Get sequence
        sequence = parser_obj.get_sequence(args.sequence)
        if variables:
            sequence = parser_obj.substitute_variables(sequence, variables)
        
        logger.info(f'Running sequence: {args.sequence}')
        logger.info(f'Duration: {sequence.get("total_duration_ms", 0)}ms')
        
        # Play
        choreography.play(sequence)
        
        # Wait for completion
        import time
        while choreography.state.value not in ['completed', 'stopped', 'error']:
            time.sleep(0.1)
        
        status = choreography.get_status()
        logger.info(f'Sequence completed: {status["state"]}')
        
        return 0
    
    parser.print_help()
    return 0


if __name__ == '__main__':
    sys.exit(main())
