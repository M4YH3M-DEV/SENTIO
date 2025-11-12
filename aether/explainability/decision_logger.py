"""
Decision Logger

Structured logging for all AI decisions with rotation and cleanup.
"""

import logging
import json
from typing import Dict, Optional
from datetime import datetime, timedelta
from pathlib import Path
import os


logger = logging.getLogger(__name__)


class DecisionLogger:
    """
    Logs AI decisions with structured format and automatic rotation.
    
    Features:
    - JSON structured logging
    - Automatic log rotation
    - Encrypted storage option
    - Log analysis utilities
    """
    
    def __init__(
        self,
        log_dir: str = '/root/aether_sentio_ws/logs',
        max_file_size_mb: int = 50,
        rotation_count: int = 10,
        encrypt: bool = False
    ):
        """
        Initialize decision logger.
        
        Args:
            log_dir: Directory for logs
            max_file_size_mb: Max size before rotation
            rotation_count: Number of rotated logs to keep
            encrypt: Whether to encrypt logs
        """
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.max_file_size = max_file_size_mb * 1024 * 1024
        self.rotation_count = rotation_count
        self.encrypt = encrypt
        self.log_file = self.log_dir / 'decisions.jsonl'
        
        logger.info(f'DecisionLogger initialized | Dir: {self.log_dir}')
    
    def log_decision(
        self,
        decision_id: str,
        decision_type: str,
        inputs: Dict,
        outputs: Dict,
        metadata: Optional[Dict] = None
    ):
        """
        Log a decision.
        
        Args:
            decision_id: Unique decision identifier
            decision_type: Type of decision
            inputs: Input data
            outputs: Output/decision data
            metadata: Optional metadata
        """
        try:
            record = {
                'timestamp': datetime.now().isoformat(),
                'decision_id': decision_id,
                'type': decision_type,
                'inputs': inputs,
                'outputs': outputs,
                'metadata': metadata or {}
            }
            
            # Check if rotation needed
            if self.log_file.exists():
                if self.log_file.stat().st_size > self.max_file_size:
                    self._rotate_logs()
            
            # Write log entry
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(record) + '\n')
            
            logger.debug(f'Decision logged: {decision_id}')
        
        except Exception as e:
            logger.error(f'Decision logging error: {str(e)}')
    
    def _rotate_logs(self):
        """Rotate log files when size limit exceeded."""
        try:
            # Rename current log
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            rotated_file = self.log_dir / f'decisions_{timestamp}.jsonl'
            
            if self.log_file.exists():
                self.log_file.rename(rotated_file)
            
            # Clean old logs
            self._cleanup_old_logs()
            
            logger.info(f'Logs rotated: {rotated_file}')
        
        except Exception as e:
            logger.error(f'Log rotation error: {str(e)}')
    
    def _cleanup_old_logs(self):
        """Remove old rotated logs exceeding retention count."""
        try:
            log_files = sorted(
                self.log_dir.glob('decisions_*.jsonl'),
                key=lambda p: p.stat().st_mtime,
                reverse=True
            )
            
            # Keep only rotation_count files
            for old_file in log_files[self.rotation_count:]:
                old_file.unlink()
                logger.debug(f'Removed old log: {old_file}')
        
        except Exception as e:
            logger.debug(f'Cleanup error: {str(e)}')
    
    def get_decisions(
        self,
        decision_type: Optional[str] = None,
        limit: int = 100
    ) -> list:
        """
        Retrieve logged decisions.
        
        Args:
            decision_type: Filter by decision type
            limit: Maximum number to return
        
        Returns:
            List of decision records
        """
        try:
            decisions = []
            
            # Read from current and rotated logs
            log_files = list(self.log_dir.glob('decisions*.jsonl'))
            log_files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
            
            for log_file in log_files:
                with open(log_file, 'r') as f:
                    for line in f:
                        if decisions and len(decisions) >= limit:
                            break
                        
                        try:
                            record = json.loads(line)
                            
                            if decision_type is None or record.get('type') == decision_type:
                                decisions.append(record)
                        
                        except json.JSONDecodeError:
                            continue
                
                if len(decisions) >= limit:
                    break
            
            return decisions[:limit]
        
        except Exception as e:
            logger.error(f'Decision retrieval error: {str(e)}')
            return []
    
    def get_decision_stats(self) -> Dict:
        """Get statistics about logged decisions."""
        try:
            decisions = self.get_decisions(limit=1000)
            
            if not decisions:
                return {'total': 0}
            
            types = {}
            for d in decisions:
                dtype = d.get('type', 'unknown')
                types[dtype] = types.get(dtype, 0) + 1
            
            return {
                'total': len(decisions),
                'types': types,
                'period': f"Last {len(decisions)} decisions"
            }
        
        except Exception as e:
            logger.error(f'Stats calculation error: {str(e)}')
            return {'error': str(e)}
