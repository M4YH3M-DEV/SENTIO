"""
Model Manifest Manager

Tracks model versions, dependencies, and metadata.
"""

import json
import logging
from pathlib import Path
from typing import Dict, Optional


logger = logging.getLogger(__name__)


class ModelManifest:
    """Manages ONNX model metadata and versioning."""
    
    def __init__(self, manifest_path: str = '/root/aether_sentio_ws/models/manifest.json'):
        """
        Initialize manifest manager.
        
        Args:
            manifest_path: Path to manifest JSON file
        """
        self.manifest_path = Path(manifest_path)
        self.manifest: Dict = {}
        
        self._load_manifest()
    
    def _load_manifest(self):
        """Load manifest from file."""
        try:
            if self.manifest_path.exists():
                with open(self.manifest_path, 'r') as f:
                    self.manifest = json.load(f)
                logger.info(f'Loaded manifest from {self.manifest_path}')
            else:
                logger.warning(f'Manifest not found at {self.manifest_path}')
                self.manifest = self._create_default()
        
        except Exception as e:
            logger.error(f'Failed to load manifest: {str(e)}')
            self.manifest = self._create_default()
    
    def _create_default(self) -> Dict:
        """Create default manifest structure."""
        return {
            'version': '1.0.0',
            'timestamp': '2025-11-12',
            'models': {
                'face_emotion': {
                    'version': '1.0.0',
                    'format': 'onnx',
                    'input_size': [1, 3, 224, 224],
                    'output_labels': 7,
                    'framework': 'pytorch',
                    'description': 'Facial expression emotion classification'
                },
                'audio_affect': {
                    'version': '1.0.0',
                    'format': 'onnx',
                    'input_features': 40,
                    'output_labels': 4,
                    'framework': 'pytorch',
                    'description': 'Speech emotion recognition'
                }
            }
        }
    
    def get_model_info(self, model_name: str) -> Optional[Dict]:
        """Get metadata for a model."""
        return self.manifest.get('models', {}).get(model_name)
    
    def save_manifest(self):
        """Save manifest to file."""
        try:
            self.manifest_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.manifest_path, 'w') as f:
                json.dump(self.manifest, f, indent=2)
            logger.info(f'Manifest saved to {self.manifest_path}')
        except Exception as e:
            logger.error(f'Failed to save manifest: {str(e)}')
