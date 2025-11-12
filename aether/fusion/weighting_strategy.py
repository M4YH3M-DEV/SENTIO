"""
Weighting Strategies for Multimodal Fusion

Implements different fusion weighting approaches based on context and confidence.
"""

import logging
from typing import Dict, Tuple
from enum import Enum


logger = logging.getLogger(__name__)


class FusionStrategy(Enum):
    """Fusion weighting strategies."""
    EQUAL_WEIGHT = 'equal_weight'
    CONFIDENCE_WEIGHTED = 'confidence_weighted'
    RECENCY_WEIGHTED = 'recency_weighted'
    ADAPTIVE = 'adaptive'


class WeightingStrategy:
    """
    Manages different strategies for weighting fusion inputs.
    
    Balances between vision and audio modalities based on confidence,
    recency, and adaptive heuristics.
    """
    
    def __init__(self, strategy: str = 'confidence_weighted'):
        """
        Initialize weighting strategy.
        
        Args:
            strategy: Strategy name (from FusionStrategy enum)
        """
        try:
            self.strategy = FusionStrategy(strategy)
        except ValueError:
            logger.warning(f'Unknown strategy {strategy}, using equal_weight')
            self.strategy = FusionStrategy.EQUAL_WEIGHT
        
        # Configuration for different strategies
        self.vision_base_weight = 0.4
        self.audio_base_weight = 0.4
        self.context_weight = 0.2
        
        logger.info(f'WeightingStrategy initialized: {self.strategy.value}')
    
    def compute_weights(
        self,
        vision_confidence: float,
        audio_confidence: float,
        context: Dict = None
    ) -> Tuple[float, float]:
        """
        Compute fusion weights for vision and audio.
        
        Args:
            vision_confidence: Confidence in vision emotion (0-1)
            audio_confidence: Confidence in audio emotion (0-1)
            context: Optional context dictionary
        
        Returns:
            Tuple of (vision_weight, audio_weight) normalized to sum 1.0
        """
        if self.strategy == FusionStrategy.EQUAL_WEIGHT:
            return 0.5, 0.5
        
        elif self.strategy == FusionStrategy.CONFIDENCE_WEIGHTED:
            return self._confidence_weighted(vision_confidence, audio_confidence)
        
        elif self.strategy == FusionStrategy.RECENCY_WEIGHTED:
            return self._recency_weighted(context)
        
        elif self.strategy == FusionStrategy.ADAPTIVE:
            return self._adaptive_weighted(
                vision_confidence,
                audio_confidence,
                context
            )
        
        return 0.5, 0.5
    
    def _confidence_weighted(
        self,
        vision_conf: float,
        audio_conf: float
    ) -> Tuple[float, float]:
        """
        Weight by confidence scores.
        
        Args:
            vision_conf: Vision confidence (0-1)
            audio_conf: Audio confidence (0-1)
        
        Returns:
            Normalized weights
        """
        # Clamp confidences to valid range
        vision_conf = max(0.0, min(1.0, vision_conf))
        audio_conf = max(0.0, min(1.0, audio_conf))
        
        # If both zero, use equal weights
        total = vision_conf + audio_conf
        if total == 0:
            return 0.5, 0.5
        
        vision_weight = vision_conf / total
        audio_weight = audio_conf / total
        
        return vision_weight, audio_weight
    
    def _recency_weighted(self, context: Dict = None) -> Tuple[float, float]:
        """
        Weight by recency of updates.
        
        Args:
            context: Context with timing information
        
        Returns:
            Normalized weights
        """
        if not context:
            return 0.5, 0.5
        
        vision_age = context.get('vision_age_ms', 0)
        audio_age = context.get('audio_age_ms', 0)
        
        # Exponential decay: more recent = higher weight
        vision_weight = 1.0 / (1.0 + vision_age / 1000.0)
        audio_weight = 1.0 / (1.0 + audio_age / 1000.0)
        
        # Normalize
        total = vision_weight + audio_weight
        if total > 0:
            vision_weight /= total
            audio_weight /= total
        else:
            vision_weight, audio_weight = 0.5, 0.5
        
        return vision_weight, audio_weight
    
    def _adaptive_weighted(
        self,
        vision_conf: float,
        audio_conf: float,
        context: Dict = None
    ) -> Tuple[float, float]:
        """
        Adaptive weighting combining multiple factors.
        
        Args:
            vision_conf: Vision confidence (0-1)
            audio_conf: Audio confidence (0-1)
            context: Context dictionary
        
        Returns:
            Normalized weights
        """
        # Start with confidence-based weights
        vision_weight, audio_weight = self._confidence_weighted(
            vision_conf, audio_conf
        )
        
        # Adjust with context if available
        if context:
            # Face detection affects vision weight
            if 'face_detected' in context:
                if context['face_detected']:
                    vision_weight *= 1.1
                else:
                    vision_weight *= 0.9
            
            # Speech activity affects audio weight
            if 'speech_active' in context:
                if context['speech_active']:
                    audio_weight *= 1.1
                else:
                    audio_weight *= 0.9
        
        # Normalize
        total = vision_weight + audio_weight
        if total > 0:
            vision_weight /= total
            audio_weight /= total
        else:
            vision_weight, audio_weight = 0.5, 0.5
        
        return vision_weight, audio_weight
    
    def get_strategy_name(self) -> str:
        """Get current strategy name."""
        return self.strategy.value
    
    def set_strategy(self, strategy: str) -> bool:
        """
        Change fusion strategy.
        
        Args:
            strategy: Strategy name
        
        Returns:
            True if successful
        """
        try:
            self.strategy = FusionStrategy(strategy)
            logger.info(f'Strategy changed to: {strategy}')
            return True
        except ValueError:
            logger.error(f'Unknown strategy: {strategy}')
            return False
