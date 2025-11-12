"""
Fusion Engine

Stateful fusion engine with history and temporal processing.
"""

import logging
import numpy as np
from typing import Dict, Optional, Deque
from collections import deque
from datetime import datetime


logger = logging.getLogger(__name__)


class FusionEngine:
    """
    Manages stateful multimodal fusion with history tracking.
    
    Features:
    - Temporal history buffer
    - Consistency checking
    - Anomaly detection
    - Trend analysis
    """
    
    def __init__(self, history_size: int = 30):
        """
        Initialize fusion engine.
        
        Args:
            history_size: Number of fusion results to keep in history
        """
        self.history_size = history_size
        self.fusion_history: Deque = deque(maxlen=history_size)
        self.last_fused_affect: Optional[Dict] = None
        self.consistency_score = 0.0
        
        logger.info(f'FusionEngine initialized with history size {history_size}')
    
    def process_inputs(
        self,
        vision_affect: Dict,
        audio_affect: Dict,
        use_history: bool = True
    ) -> Dict:
        """
        Process and fuse multimodal inputs with history.
        
        Args:
            vision_affect: Vision emotion dict
            audio_affect: Audio emotion dict
            use_history: Whether to use history for smoothing
        
        Returns:
            Fused affect dictionary with metadata
        """
        try:
            # Import here to avoid circular imports
            from .fuse_affect import fuse_affect
            
            # Perform fusion
            fused, reasoning = fuse_affect(vision_affect, audio_affect)
            
            if not fused:
                return {}
            
            # Apply historical smoothing if enabled
            if use_history and self.last_fused_affect:
                fused = self._smooth_with_history(fused)
            
            # Add timestamp
            fused['timestamp'] = datetime.now().isoformat()
            fused['reasoning'] = reasoning
            
            # Store in history
            self.fusion_history.append(fused)
            self.last_fused_affect = fused
            
            # Calculate consistency
            self.consistency_score = self._calculate_consistency()
            fused['consistency_score'] = self.consistency_score
            
            return fused
        
        except Exception as e:
            logger.error(f'Fusion processing error: {str(e)}')
            return {}
    
    def _smooth_with_history(self, current_fused: Dict) -> Dict:
        """
        Smooth current fusion result with history.
        
        Args:
            current_fused: Current fused affect
        
        Returns:
            Smoothed affect dict
        """
        try:
            alpha = 0.2  # Smoothing factor
            
            prev = self.last_fused_affect
            current = current_fused
            
            # EMA for valence and arousal
            smoothed_valence = (
                alpha * current.get('valence', 0) +
                (1 - alpha) * prev.get('valence', 0)
            )
            smoothed_arousal = (
                alpha * current.get('arousal', 0.3) +
                (1 - alpha) * prev.get('arousal', 0.3)
            )
            
            current['valence'] = float(smoothed_valence)
            current['arousal'] = float(smoothed_arousal)
            
            return current
        
        except Exception as e:
            logger.debug(f'History smoothing error: {str(e)}')
            return current_fused
    
    def _calculate_consistency(self) -> float:
        """
        Calculate temporal consistency score.
        
        Returns:
            Consistency score (0-1)
        """
        try:
            if len(self.fusion_history) < 2:
                return 1.0
            
            # Compare last two results
            recent_history = list(self.fusion_history)[-5:]
            
            if len(recent_history) < 2:
                return 1.0
            
            # Calculate variance in VA space
            valences = [h.get('valence', 0) for h in recent_history]
            arousals = [h.get('arousal', 0.3) for h in recent_history]
            
            val_var = np.var(valences)
            arou_var = np.var(arousals)
            
            # Higher variance = lower consistency
            consistency = 1.0 / (1.0 + val_var + arou_var)
            
            return float(np.clip(consistency, 0.0, 1.0))
        
        except Exception as e:
            logger.debug(f'Consistency calculation error: {str(e)}')
            return 0.5
    
    def get_trend(self, window: int = 5) -> Dict:
        """
        Get emotion trend over recent window.
        
        Args:
            window: Number of recent samples to analyze
        
        Returns:
            Trend analysis dictionary
        """
        try:
            recent = list(self.fusion_history)[-window:]
            
            if len(recent) < 2:
                return {'trend': 'insufficient_data'}
            
            valences = [h.get('valence', 0) for h in recent]
            arousals = [h.get('arousal', 0.3) for h in recent]
            
            # Simple linear regression for trend
            x = np.arange(len(valences))
            val_slope = float(np.polyfit(x, valences, 1)[0])
            arou_slope = float(np.polyfit(x, arousals, 1)[0])
            
            return {
                'valence_trend': 'increasing' if val_slope > 0.05 else 'decreasing' if val_slope < -0.05 else 'stable',
                'valence_slope': val_slope,
                'arousal_trend': 'increasing' if arou_slope > 0.05 else 'decreasing' if arou_slope < -0.05 else 'stable',
                'arousal_slope': arou_slope,
                'window': window
            }
        
        except Exception as e:
            logger.debug(f'Trend calculation error: {str(e)}')
            return {'error': str(e)}
    
    def detect_anomalies(self, threshold: float = 2.0) -> Dict:
        """
        Detect anomalous affect values.
        
        Args:
            threshold: Z-score threshold for anomaly
        
        Returns:
            Anomaly detection results
        """
        try:
            if len(self.fusion_history) < 3:
                return {'anomalies': 0}
            
            recent = list(self.fusion_history)[-10:]
            valences = [h.get('valence', 0) for h in recent]
            arousals = [h.get('arousal', 0.3) for h in recent]
            
            # Z-score based anomaly detection
            val_mean = np.mean(valences)
            val_std = np.std(valences) + 1e-6
            arou_mean = np.mean(arousals)
            arou_std = np.std(arousals) + 1e-6
            
            last_val = valences[-1]
            last_arou = arousals[-1]
            
            val_zscore = abs((last_val - val_mean) / val_std)
            arou_zscore = abs((last_arou - arou_mean) / arou_std)
            
            is_anomalous = val_zscore > threshold or arou_zscore > threshold
            
            return {
                'is_anomalous': is_anomalous,
                'valence_zscore': float(val_zscore),
                'arousal_zscore': float(arou_zscore),
                'threshold': threshold
            }
        
        except Exception as e:
            logger.debug(f'Anomaly detection error: {str(e)}')
            return {'error': str(e)}
    
    def reset_history(self):
        """Clear fusion history."""
        self.fusion_history.clear()
        self.last_fused_affect = None
        self.consistency_score = 0.0
        logger.info('Fusion history cleared')
