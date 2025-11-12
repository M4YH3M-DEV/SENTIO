"""Choreography sequence YAML parser."""

import yaml
import logging
from typing import Dict, List, Optional, Any
from pathlib import Path


logger = logging.getLogger(__name__)


class SequenceParser:
    """
    Parses YAML choreography sequences.
    
    Sequence format:
    ```
    name: "Sequence Name"
    description: "Description"
    steps:
      - action: "gesture"
        gesture: "greet"
        delay_ms: 0
        duration_ms: 1000
      - action: "tts"
        text: "Hello {visitor_name}!"
        delay_ms: 500
      - action: "wait"
        duration_ms: 2000
    ```
    """
    
    def __init__(self):
        """Initialize parser."""
        self.sequences: Dict[str, Dict] = {}
    
    def load_sequence(self, yaml_file: str) -> Optional[Dict]:
        """
        Load sequence from YAML file.
        
        Args:
            yaml_file: Path to YAML file
        
        Returns:
            Parsed sequence dict or None if error
        """
        try:
            yaml_path = Path(yaml_file)
            
            if not yaml_path.exists():
                logger.error(f'Sequence file not found: {yaml_file}')
                return None
            
            with open(yaml_path, 'r') as f:
                sequence = yaml.safe_load(f)
            
            # Validate sequence
            if not self._validate_sequence(sequence):
                return None
            
            # Parse sequence
            parsed = self._parse_sequence(sequence)
            
            # Store by name
            seq_name = parsed.get('name', 'unnamed')
            self.sequences[seq_name] = parsed
            
            logger.info(f'Loaded sequence: {seq_name} with {len(parsed["steps"])} steps')
            
            return parsed
        
        except yaml.YAMLError as e:
            logger.error(f'YAML parse error: {str(e)}')
            return None
        except Exception as e:
            logger.error(f'Failed to load sequence: {str(e)}')
            return None
    
    def _validate_sequence(self, sequence: Dict) -> bool:
        """Validate sequence structure."""
        if not isinstance(sequence, dict):
            logger.error('Sequence must be a dict')
            return False
        
        if 'steps' not in sequence:
            logger.error('Sequence missing "steps" key')
            return False
        
        if not isinstance(sequence['steps'], list):
            logger.error('Steps must be a list')
            return False
        
        for i, step in enumerate(sequence['steps']):
            if not isinstance(step, dict):
                logger.error(f'Step {i} is not a dict')
                return False
            
            if 'action' not in step:
                logger.error(f'Step {i} missing "action" key')
                return False
        
        return True
    
    def _parse_sequence(self, sequence: Dict) -> Dict:
        """Parse sequence structure."""
        parsed = {
            'name': sequence.get('name', 'unnamed'),
            'description': sequence.get('description', ''),
            'steps': [],
            'total_duration_ms': 0,
        }
        
        current_time_ms = 0
        
        for step_idx, step in enumerate(sequence.get('steps', [])):
            action = step.get('action', 'unknown')
            delay_ms = step.get('delay_ms', 0)
            
            # Calculate absolute timestamp
            absolute_time_ms = current_time_ms + delay_ms
            
            parsed_step = {
                'index': step_idx,
                'action': action,
                'timestamp_ms': absolute_time_ms,
                'delay_ms': delay_ms,
                'data': step,
            }
            
            parsed['steps'].append(parsed_step)
            
            # Update timeline
            if action == 'wait':
                current_time_ms = absolute_time_ms + step.get('duration_ms', 0)
            elif action == 'gesture':
                current_time_ms = absolute_time_ms + step.get('duration_ms', 1000)
            elif action == 'tts':
                current_time_ms = absolute_time_ms + 1000  # Approximate
            else:
                current_time_ms = absolute_time_ms + 100
        
        parsed['total_duration_ms'] = current_time_ms
        
        return parsed
    
    def substitute_variables(
        self,
        sequence: Dict,
        variables: Optional[Dict[str, Any]] = None
    ) -> Dict:
        """
        Substitute variables in sequence.
        
        Args:
            sequence: Parsed sequence
            variables: Dict of variables to substitute
        
        Returns:
            Sequence with substituted values
        """
        if not variables:
            return sequence
        
        # Deep copy sequence
        import copy
        result = copy.deepcopy(sequence)
        
        # Substitute in steps
        for step in result.get('steps', []):
            step_data = step.get('data', {})
            
            if 'text' in step_data:
                try:
                    step_data['text'] = step_data['text'].format(**variables)
                except KeyError as e:
                    logger.warning(f'Missing variable: {str(e)}')
        
        return result
    
    def get_sequence(self, name: str) -> Optional[Dict]:
        """Get loaded sequence by name."""
        return self.sequences.get(name)
    
    def list_sequences(self) -> List[str]:
        """List all loaded sequence names."""
        return list(self.sequences.keys())
