"""
Multimodal Affect Fusion

Core fusion function combining vision and audio emotions into unified affect.
"""

import logging
from typing import Dict, Tuple, Optional
import numpy as np


logger = logging.getLogger(__name__)


def fuse_affect(
    vision_affect: Dict,
    audio_affect: Dict,
    context: Optional[Dict] = None,
    fusion_weights: Optional[Dict] = None
) -> Tuple[Dict, Dict]:
    """
    Fuse vision and audio affect into unified representation.
    
    Args:
        vision_affect: Vision affect dict with 'valence', 'arousal', 'confidence'
        audio_affect: Audio affect dict with 'valence', 'arousal', 'confidence'
        context: Optional context information
        fusion_weights: Optional weights {'vision': float, 'audio': float}
    
    Returns:
        Tuple of (fused_affect, reasoning_dict)
    
    Example:
        vision = {'valence': 0.8, 'arousal': 0.5, 'confidence': 0.9}
        audio = {'valence': -0.2, 'arousal': 0.6, 'confidence': 0.7}
        fused, reasoning = fuse_affect(vision, audio)
        # fused: unified valence-arousal
        # reasoning: explains the fusion decision
    """
    try:
        # Validate inputs
        if not vision_affect or not audio_affect:
            logger.error('Missing vision or audio affect')
            return {}, {}
        
        # Extract values
        v_val = float(vision_affect.get('valence', 0.0))
        v_arou = float(vision_affect.get('arousal', 0.3))
        v_conf = float(vision_affect.get('confidence', 0.5))
        
        a_val = float(audio_affect.get('valence', 0.0))
        a_arou = float(audio_affect.get('arousal', 0.3))
        a_conf = float(audio_affect.get('confidence', 0.5))
        
        # Set default weights if not provided
        if not fusion_weights:
            fusion_weights = _compute_default_weights(v_conf, a_conf)
        
        v_weight = fusion_weights.get('vision', 0.5)
        a_weight = fusion_weights.get('audio', 0.5)
        
        # Normalize weights
        total_weight = v_weight + a_weight
        if total_weight > 0:
            v_weight /= total_weight
            a_weight /= total_weight
        else:
            v_weight, a_weight = 0.5, 0.5
        
        # Weighted fusion
        fused_valence = v_weight * v_val + a_weight * a_val
        fused_arousal = v_weight * v_arou + a_weight * a_arou
        
        # Combined confidence (product of individual confidences)
        fused_confidence = np.sqrt(v_conf * a_conf)
        
        # Clamp to valid ranges
        fused_valence = np.clip(fused_valence, -1.0, 1.0)
        fused_arousal = np.clip(fused_arousal, 0.0, 1.0)
        
        # Create fused affect dict
        fused_affect = {
            'valence': float(fused_valence),
            'arousal': float(fused_arousal),
            'confidence': float(fused_confidence),
            'source': 'fusion',
            'modalities': 2
        }
        
        # Create reasoning dict
        reasoning = {
            'vision_contribution': float(v_weight),
            'audio_contribution': float(a_weight),
            'valence_shift': float(fused_valence - (v_val + a_val) / 2),
            'arousal_shift': float(fused_arousal - (v_arou + a_arou) / 2),
            'confidence_source': {
                'vision': float(v_conf),
                'audio': float(a_conf),
                'fused': float(fused_confidence)
            },
            'disagreement_level': float(_calculate_disagreement(
                (v_val, v_arou), (a_val, a_arou)
            ))
        }
        
        logger.debug(
            f'Fused affect: VA({fused_valence:.2f}, {fused_arousal:.2f}) | '
            f'Weights: V{v_weight:.2f} A{a_weight:.2f}'
        )
        
        return fused_affect, reasoning
    
    except Exception as e:
        logger.error(f'Fusion error: {str(e)}')
        return {}, {'error': str(e)}


def _compute_default_weights(vision_conf: float, audio_conf: float) -> Dict:
    """
    Compute default fusion weights based on confidences.
    
    Args:
        vision_conf: Vision confidence (0-1)
        audio_conf: Audio confidence (0-1)
    
    Returns:
        Dictionary with 'vision' and 'audio' weights
    """
    # Clamp
    vision_conf = max(0.0, min(1.0, vision_conf))
    audio_conf = max(0.0, min(1.0, audio_conf))
    
    # Add base weights
    vision_weight = 0.4 + 0.2 * vision_conf
    audio_weight = 0.4 + 0.2 * audio_conf
    
    return {'vision': vision_weight, 'audio': audio_weight}


def _calculate_disagreement(
    vision_va: Tuple[float, float],
    audio_va: Tuple[float, float]
) -> float:
    """
    Calculate disagreement between modalities in VA space.
    
    Args:
        vision_va: (valence, arousal) from vision
        audio_va: (valence, arousal) from audio
    
    Returns:
        Disagreement score (0-1)
    """
    v_val, v_arou = vision_va
    a_val, a_arou = audio_va
    
    # Euclidean distance in normalized VA space
    val_diff = abs(v_val - a_val) / 2.0  # Max diff is 2
    arou_diff = abs(v_arou - a_arou)     # Max diff is 1
    
    disagreement = np.sqrt(val_diff ** 2 + arou_diff ** 2)
    
    return float(np.clip(disagreement, 0.0, 1.0))
