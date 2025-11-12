"""
Model Loader for ONNX Inference

Manages loading and caching of ONNX models for emotion detection.
"""

import logging
from pathlib import Path
from typing import Optional, Dict, Tuple
import json


logger = logging.getLogger(__name__)


class ModelLoader:
    """
    Loads and manages ONNX models for inference.
    
    Supports multiple model formats and fallback logic.
    """
    
    def __init__(self, models_dir: str = '/root/aether_sentio_ws/models'):
        """
        Initialize model loader.
        
        Args:
            models_dir: Directory containing ONNX models
        """
        self.models_dir = Path(models_dir)
        self.models_dir.mkdir(parents=True, exist_ok=True)
        
        self.models: Dict[str, Dict] = {}
        self.onnx_runtime_available = False
        
        # Try to import onnxruntime
        try:
            import onnxruntime as ort
            self.ort = ort
            self.onnx_runtime_available = True
            logger.info('ONNX Runtime available')
        except ImportError:
            logger.warning('ONNX Runtime not available, using mock inference')
            self.ort = None
        
        self._load_manifest()
    
    def _load_manifest(self):
        """Load model manifest from JSON."""
        manifest_path = self.models_dir / 'manifest.json'
        
        if manifest_path.exists():
            try:
                with open(manifest_path, 'r') as f:
                    self.models = json.load(f)
                logger.info(f'Loaded manifest with {len(self.models)} models')
            except Exception as e:
                logger.warning(f'Failed to load manifest: {str(e)}')
                self.models = self._create_default_manifest()
        else:
            logger.info('No manifest found, using defaults')
            self.models = self._create_default_manifest()
    
    def _create_default_manifest(self) -> Dict:
        """Create default model manifest."""
        return {
            'face_emotion': {
                'filename': 'face_emotion.onnx',
                'type': 'facial_expression',
                'input_shape': [1, 3, 224, 224],
                'output_labels': ['neutral', 'happy', 'sad', 'angry', 
                                 'fearful', 'disgusted', 'surprised'],
                'providers': ['CPUExecutionProvider'],
            },
            'audio_affect': {
                'filename': 'audio_affect.onnx',
                'type': 'speech_emotion',
                'input_features': 40,  # Mel spectrogram features
                'output_labels': ['neutral', 'happy', 'sad', 'angry'],
                'providers': ['CPUExecutionProvider'],
            },
        }
    
    def load_model(self, model_name: str) -> Optional[object]:
        """
        Load an ONNX model session.
        
        Args:
            model_name: Model identifier
        
        Returns:
            ONNX Runtime InferenceSession or None
        """
        if model_name not in self.models:
            logger.error(f'Model not in manifest: {model_name}')
            return None
        
        model_info = self.models[model_name]
        model_path = self.models_dir / model_info['filename']
        
        if not model_path.exists():
            logger.error(f'Model file not found: {model_path}')
            return None
        
        if not self.onnx_runtime_available:
            logger.warning(f'ONNX Runtime not available, returning mock session')
            return MockONNXSession(model_name, model_info)
        
        try:
            session = self.ort.InferenceSession(
                str(model_path),
                providers=model_info.get('providers', ['CPUExecutionProvider'])
            )
            logger.info(f'Loaded model: {model_name} from {model_path}')
            return session
        
        except Exception as e:
            logger.error(f'Failed to load model {model_name}: {str(e)}')
            return None
    
    def get_model_info(self, model_name: str) -> Optional[Dict]:
        """Get model metadata."""
        return self.models.get(model_name)


class MockONNXSession:
    """
    Mock ONNX session for testing when real ONNX Runtime is unavailable.
    
    Returns deterministic but realistic-looking inference results.
    """
    
    def __init__(self, model_name: str, model_info: Dict):
        """Initialize mock session."""
        self.model_name = model_name
        self.model_info = model_info
        self.counter = 0
    
    def run(self, output_names: list, input_feed: Dict) -> Tuple:
        """
        Run mock inference.
        
        Args:
            output_names: Output tensor names
            input_feed: Input dictionary
        
        Returns:
            Tuple of output arrays
        """
        if 'face' in self.model_name or 'facial' in str(self.model_info.get('type', '')):
            return self._mock_face_inference(output_names)
        elif 'audio' in self.model_name or 'speech' in str(self.model_info.get('type', '')):
            return self._mock_audio_inference(output_names)
        else:
            return self._mock_generic_inference(output_names)
    
    def _mock_face_inference(self, output_names: list) -> Tuple:
        """Mock facial expression inference."""
        import numpy as np
        
        # Oscillate through emotions
        emotions = ['neutral', 'happy', 'sad', 'angry', 'fearful', 'disgusted', 'surprised']
        logits = np.zeros(len(emotions), dtype=np.float32)
        
        emotion_idx = self.counter % len(emotions)
        logits[emotion_idx] = 0.8
        logits[(emotion_idx + 1) % len(emotions)] = 0.15
        
        self.counter += 1
        
        return (logits.reshape(1, -1),)
    
    def _mock_audio_inference(self, output_names: list) -> Tuple:
        """Mock audio affect inference."""
        import numpy as np
        
        emotions = ['neutral', 'happy', 'sad', 'angry']
        logits = np.zeros(len(emotions), dtype=np.float32)
        
        emotion_idx = self.counter % len(emotions)
        logits[emotion_idx] = 0.7
        logits[(emotion_idx + 1) % len(emotions)] = 0.2
        
        self.counter += 1
        
        return (logits.reshape(1, -1),)
    
    def _mock_generic_inference(self, output_names: list) -> Tuple:
        """Mock generic inference."""
        import numpy as np
        
        # Return dummy output
        output_count = len(output_names) if output_names else 1
        return tuple(np.random.rand(1, 10).astype(np.float32) 
                    for _ in range(output_count))
