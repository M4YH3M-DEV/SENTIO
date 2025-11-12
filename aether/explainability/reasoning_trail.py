"""
Reasoning Trail

Captures and logs the decision-making process for transparency.
"""

import logging
import json
from typing import Dict, List, Any, Optional
from datetime import datetime
from pathlib import Path


logger = logging.getLogger(__name__)


class ReasoningTrail:
    """
    Captures detailed reasoning trails for AI decisions.
    
    Records:
    - What inputs were observed
    - How models scored them
    - How fusion weights were computed
    - Final decision and confidence
    - Explainability metadata
    """
    
    def __init__(self, log_dir: str = '/root/aether_sentio_ws/logs'):
        """
        Initialize reasoning trail.
        
        Args:
            log_dir: Directory for trail logs
        """
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.current_trail: List[Dict] = []
        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        logger.info(f'ReasoningTrail initialized | Log dir: {self.log_dir}')
    
    def start_decision_chain(self, decision_type: str, context: Optional[Dict] = None):
        """
        Start recording a new decision chain.
        
        Args:
            decision_type: Type of decision (e.g., 'affect_fusion')
            context: Optional context information
        """
        self.current_trail = []
        
        self.add_entry(
            'chain_start',
            {
                'type': decision_type,
                'context': context or {},
                'timestamp': datetime.now().isoformat()
            }
        )
    
    def add_entry(
        self,
        entry_type: str,
        data: Dict,
        confidence: Optional[float] = None,
        reasoning: Optional[str] = None
    ):
        """
        Add entry to current reasoning trail.
        
        Args:
            entry_type: Type of entry (e.g., 'perception', 'fusion', 'decision')
            data: Entry data dictionary
            confidence: Optional confidence score (0-1)
            reasoning: Optional human-readable reasoning
        """
        entry = {
            'timestamp': datetime.now().isoformat(),
            'type': entry_type,
            'data': data,
            'sequence': len(self.current_trail)
        }
        
        if confidence is not None:
            entry['confidence'] = float(confidence)
        
        if reasoning:
            entry['reasoning'] = reasoning
        
        self.current_trail.append(entry)
    
    def end_decision_chain(self, decision: Dict, outcome: Optional[str] = None) -> str:
        """
        End current decision chain and save to file.
        
        Args:
            decision: Final decision dictionary
            outcome: Optional outcome description
        
        Returns:
            Trail ID (filename)
        """
        try:
            self.add_entry(
                'chain_end',
                {
                    'decision': decision,
                    'outcome': outcome,
                    'timestamp': datetime.now().isoformat()
                }
            )
            
            # Save trail to file
            trail_id = f'trail_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}'
            trail_path = self.log_dir / f'{trail_id}.json'
            
            with open(trail_path, 'w') as f:
                json.dump(self.current_trail, f, indent=2)
            
            logger.debug(f'Reasoning trail saved: {trail_path}')
            
            self.current_trail = []
            return trail_id
        
        except Exception as e:
            logger.error(f'Failed to save reasoning trail: {str(e)}')
            return ''
    
    def get_current_trail(self) -> List[Dict]:
        """Get current trail entries."""
        return list(self.current_trail)
    
    def create_explanation(self) -> Dict:
        """
        Generate human-readable explanation from current trail.
        
        Returns:
            Explanation dictionary
        """
        if not self.current_trail:
            return {'error': 'No trail data'}
        
        try:
            explanation = {
                'decision_type': None,
                'steps': [],
                'final_decision': None,
                'confidence': None
            }
            
            for entry in self.current_trail:
                if entry['type'] == 'chain_start':
                    explanation['decision_type'] = entry['data'].get('type')
                
                elif entry['type'] == 'chain_end':
                    explanation['final_decision'] = entry['data'].get('decision')
                
                elif entry['type'] in ['perception', 'fusion', 'decision']:
                    step_desc = f"{entry['type']}: {entry['data']}"
                    if 'confidence' in entry:
                        step_desc += f" (confidence: {entry['confidence']:.2f})"
                    if 'reasoning' in entry:
                        step_desc += f" - {entry['reasoning']}"
                    
                    explanation['steps'].append(step_desc)
            
            return explanation
        
        except Exception as e:
            logger.error(f'Explanation creation error: {str(e)}')
            return {'error': str(e)}
    
    def list_trails(self, limit: int = 10) -> List[Dict]:
        """
        List recent reasoning trails.
        
        Args:
            limit: Maximum number of trails to return
        
        Returns:
            List of trail metadata
        """
        try:
            trails = sorted(
                self.log_dir.glob('trail_*.json'),
                key=lambda p: p.stat().st_mtime,
                reverse=True
            )[:limit]
            
            trail_info = []
            for trail_path in trails:
                with open(trail_path, 'r') as f:
                    trail_data = json.load(f)
                
                if trail_data:
                    trail_info.append({
                        'filename': trail_path.name,
                        'entries': len(trail_data),
                        'start_time': trail_data[0].get('timestamp') if trail_data else None
                    })
            
            return trail_info
        
        except Exception as e:
            logger.error(f'Trail listing error: {str(e)}')
            return []
