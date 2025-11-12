"""Choreography execution engine."""

import logging
import time
from typing import Dict, List, Optional, Callable
from enum import Enum
from threading import Thread, Event


logger = logging.getLogger(__name__)


class PlaybackState(Enum):
    """Choreography playback states."""
    IDLE = 'idle'
    PLAYING = 'playing'
    PAUSED = 'paused'
    STOPPED = 'stopped'
    COMPLETED = 'completed'
    ERROR = 'error'


class ChoreographyEngine:
    """
    Executes choreography sequences with timing control.
    
    Manages playback, pause/resume, and callbacks for each action.
    """
    
    def __init__(self):
        """Initialize choreography engine."""
        self.state = PlaybackState.IDLE
        self.current_sequence: Optional[Dict] = None
        self.current_step_index = 0
        self.playback_start_time = 0.0
        self.pause_time = 0.0
        self.total_pause_duration = 0.0
        
        self.action_callbacks: Dict[str, Callable] = {}
        
        self.playback_thread: Optional[Thread] = None
        self.stop_event = Event()
        self.pause_event = Event()
        
        self.step_log: List[Dict] = []
    
    def register_action_callback(self, action: str, callback: Callable):
        """
        Register callback for action type.
        
        Args:
            action: Action name (e.g., 'gesture', 'tts')
            callback: Function to call with action data
        """
        self.action_callbacks[action] = callback
        logger.debug(f'Registered callback for action: {action}')
    
    def play(self, sequence: Dict):
        """
        Start playing a sequence.
        
        Args:
            sequence: Parsed sequence dict
        """
        if self.state == PlaybackState.PLAYING:
            logger.warning('Already playing, ignoring play request')
            return
        
        self.current_sequence = sequence
        self.current_step_index = 0
        self.playback_start_time = time.time()
        self.total_pause_duration = 0.0
        self.step_log = []
        self.state = PlaybackState.PLAYING
        self.stop_event.clear()
        self.pause_event.clear()
        
        # Start playback thread
        self.playback_thread = Thread(target=self._playback_loop, daemon=False)
        self.playback_thread.start()
        
        logger.info(f'Playing sequence: {sequence.get("name", "unnamed")}')
    
    def pause(self):
        """Pause playback."""
        if self.state == PlaybackState.PLAYING:
            self.state = PlaybackState.PAUSED
            self.pause_time = time.time()
            logger.info('Playback paused')
    
    def resume(self):
        """Resume playback."""
        if self.state == PlaybackState.PAUSED:
            self.total_pause_duration += time.time() - self.pause_time
            self.state = PlaybackState.PLAYING
            logger.info('Playback resumed')
    
    def stop(self):
        """Stop playback."""
        if self.state in [PlaybackState.PLAYING, PlaybackState.PAUSED]:
            self.state = PlaybackState.STOPPED
            self.stop_event.set()
            
            if self.playback_thread:
                self.playback_thread.join(timeout=2.0)
            
            logger.info('Playback stopped')
    
    def _playback_loop(self):
        """Main playback loop (runs in thread)."""
        try:
            if not self.current_sequence:
                return
            
            steps = self.current_sequence.get('steps', [])
            
            while self.current_step_index < len(steps):
                # Check stop signal
                if self.stop_event.is_set():
                    break
                
                # Handle pause
                while self.state == PlaybackState.PAUSED:
                    time.sleep(0.1)
                
                step = steps[self.current_step_index]
                self._execute_step(step)
                
                self.current_step_index += 1
                
                # Wait for next step
                if self.current_step_index < len(steps):
                    next_step = steps[self.current_step_index]
                    wait_time_ms = next_step.get('timestamp_ms', 0) - step.get('timestamp_ms', 0)
                    
                    if wait_time_ms > 0:
                        self._wait(wait_time_ms / 1000.0)
            
            # Sequence complete
            self.state = PlaybackState.COMPLETED
            logger.info('Sequence playback completed')
        
        except Exception as e:
            logger.error(f'Playback error: {str(e)}')
            self.state = PlaybackState.ERROR
    
    def _execute_step(self, step: Dict):
        """Execute a single step."""
        try:
            action = step.get('action', 'unknown')
            step_data = step.get('data', {})
            
            if action == 'wait':
                duration_ms = step_data.get('duration_ms', 0)
                logger.debug(f'Wait {duration_ms}ms')
            
            elif action in self.action_callbacks:
                callback = self.action_callbacks[action]
                callback(step_data)
            
            else:
                logger.warning(f'No handler for action: {action}')
            
            # Log step
            self.step_log.append({
                'index': step.get('index', -1),
                'action': action,
                'timestamp': time.time(),
                'status': 'executed'
            })
        
        except Exception as e:
            logger.error(f'Step execution error: {str(e)}')
            self.step_log.append({
                'index': step.get('index', -1),
                'action': step.get('action', 'unknown'),
                'timestamp': time.time(),
                'status': 'error',
                'error': str(e)
            })
    
    def _wait(self, duration_s: float):
        """Wait for duration, respecting pause/stop signals."""
        end_time = time.time() + duration_s
        
        while time.time() < end_time:
            if self.stop_event.is_set():
                break
            
            if self.state == PlaybackState.PAUSED:
                time.sleep(0.1)
            else:
                time.sleep(0.05)
    
    def get_status(self) -> Dict:
        """Get current playback status."""
        elapsed_ms = 0
        if self.playback_start_time > 0:
            elapsed_ms = int((time.time() - self.playback_start_time - self.total_pause_duration) * 1000)
        
        total_duration_ms = self.current_sequence.get('total_duration_ms', 0) if self.current_sequence else 0
        
        return {
            'state': self.state.value,
            'sequence_name': self.current_sequence.get('name', '') if self.current_sequence else '',
            'current_step': self.current_step_index,
            'total_steps': len(self.current_sequence.get('steps', [])) if self.current_sequence else 0,
            'elapsed_ms': elapsed_ms,
            'total_duration_ms': total_duration_ms,
            'step_log': self.step_log[-10:],  # Last 10 steps
        }
    
    def get_step_log(self) -> List[Dict]:
        """Get execution log."""
        return list(self.step_log)
