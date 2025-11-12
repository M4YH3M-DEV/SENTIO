"""Demo control interface."""

import logging
from enum import Enum
from typing import Dict, Optional


logger = logging.getLogger(__name__)


class DemoCommand(Enum):
    """Demo control commands."""
    START = 'start'
    STOP = 'stop'
    PAUSE = 'pause'
    RESUME = 'resume'


class DemoController:
    """
    Controls demo execution via service/action interface.
    """
    
    def __init__(self, choreography_engine, sequence_parser):
        """
        Initialize demo controller.
        
        Args:
            choreography_engine: ChoreographyEngine instance
            sequence_parser: SequenceParser instance
        """
        self.choreography = choreography_engine
        self.parser = sequence_parser
    
    def start_demo(self, sequence_name: str, variables: Optional[Dict] = None) -> bool:
        """
        Start a demo sequence.
        
        Args:
            sequence_name: Name of sequence to play
            variables: Optional variables for substitution
        
        Returns:
            True if successful
        """
        try:
            sequence = self.parser.get_sequence(sequence_name)
            
            if not sequence:
                logger.error(f'Sequence not found: {sequence_name}')
                return False
            
            # Substitute variables
            if variables:
                sequence = self.parser.substitute_variables(sequence, variables)
            
            # Play sequence
            self.choreography.play(sequence)
            
            return True
        
        except Exception as e:
            logger.error(f'Failed to start demo: {str(e)}')
            return False
    
    def stop_demo(self) -> bool:
        """Stop current demo."""
        try:
            self.choreography.stop()
            return True
        except Exception as e:
            logger.error(f'Failed to stop demo: {str(e)}')
            return False
    
    def pause_demo(self) -> bool:
        """Pause current demo."""
        try:
            self.choreography.pause()
            return True
        except Exception as e:
            logger.error(f'Failed to pause demo: {str(e)}')
            return False
    
    def resume_demo(self) -> bool:
        """Resume paused demo."""
        try:
            self.choreography.resume()
            return True
        except Exception as e:
            logger.error(f'Failed to resume demo: {str(e)}')
            return False
    
    def get_status(self) -> Dict:
        """Get demo status."""
        return self.choreography.get_status()
    
    def list_available_demos(self) -> list:
        """List available demo sequences."""
        return self.parser.list_sequences()
