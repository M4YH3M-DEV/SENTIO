"""
Explanation Formatter

Formats explainability data for human consumption and visualization.
"""

import logging
from typing import Dict, List, Any
import json


logger = logging.getLogger(__name__)


class ExplanationFormatter:
    """
    Formats AETHER decisions into human-readable explanations.
    
    Outputs:
    - Plain text explanations
    - JSON structure
    - Visualization-friendly format
    """
    
    @staticmethod
    def format_fusion_explanation(
        reasoning: Dict,
        decision: Dict
    ) -> str:
        """
        Format fusion decision explanation.
        
        Args:
            reasoning: Reasoning dictionary from fusion
            decision: Final decision dictionary
        
        Returns:
            Human-readable explanation
        """
        try:
            lines = []
            
            lines.append("=== SENTIO Affect Fusion Explanation ===\n")
            
            # Emotion result
            emotion = decision.get('emotion', 'unknown')
            valence = decision.get('valence', 0)
            arousal = decision.get('arousal', 0.3)
            
            lines.append(f"Detected Emotion: {emotion.upper()}")
            lines.append(f"Valence (negative↔positive): {valence:+.2f}")
            lines.append(f"Arousal (calm↔excited): {arousal:.2f}\n")
            
            # Modality contributions
            v_contrib = reasoning.get('vision_contribution', 0.5)
            a_contrib = reasoning.get('audio_contribution', 0.5)
            
            lines.append("Modality Analysis:")
            lines.append(f"  Vision (facial): {v_contrib*100:.0f}% → {reasoning.get('vision_emotion', 'N/A')}")
            lines.append(f"  Audio (tone): {a_contrib*100:.0f}% → {reasoning.get('audio_emotion', 'N/A')}\n")
            
            # Confidence
            conf = decision.get('confidence', 0.5)
            lines.append(f"Confidence: {conf*100:.0f}%")
            
            if conf < 0.6:
                lines.append("  ⚠ Low confidence - emotion may be ambiguous")
            elif conf > 0.8:
                lines.append("  ✓ High confidence - emotion is clear")
            
            # Disagreement
            disagreement = reasoning.get('disagreement_level', 0)
            lines.append(f"\nModality Agreement: {(1-disagreement)*100:.0f}%")
            
            if disagreement > 0.3:
                lines.append("  → Vision and audio show different emotions")
                lines.append("  → Fusion balances both signals")
            
            return '\n'.join(lines)
        
        except Exception as e:
            logger.error(f'Explanation formatting error: {str(e)}')
            return f'Error formatting explanation: {str(e)}'
    
    @staticmethod
    def format_json(data: Dict, indent: int = 2) -> str:
        """Format data as JSON."""
        try:
            return json.dumps(data, indent=indent)
        except Exception as e:
            logger.error(f'JSON formatting error: {str(e)}')
            return '{}'
    
    @staticmethod
    def format_confidence_bar(confidence: float, width: int = 20) -> str:
        """
        Create ASCII confidence bar.
        
        Args:
            confidence: Confidence value (0-1)
            width: Width of bar
        
        Returns:
            ASCII bar representation
        """
        filled = int(confidence * width)
        bar = '█' * filled + '░' * (width - filled)
        return f'[{bar}] {confidence*100:.0f}%'
    
    @staticmethod
    def format_timeline(entries: List[Dict]) -> str:
        """
        Format decision timeline.
        
        Args:
            entries: List of decision entries
        
        Returns:
            Formatted timeline
        """
        try:
            lines = ["=== Decision Timeline ===\n"]
            
            for i, entry in enumerate(entries, 1):
                timestamp = entry.get('timestamp', 'N/A')
                entry_type = entry.get('type', 'unknown')
                
                lines.append(f"{i}. [{timestamp}] {entry_type}")
                
                if 'reasoning' in entry:
                    lines.append(f"   {entry['reasoning']}")
            
            return '\n'.join(lines)
        
        except Exception as e:
            logger.error(f'Timeline formatting error: {str(e)}')
            return ''
