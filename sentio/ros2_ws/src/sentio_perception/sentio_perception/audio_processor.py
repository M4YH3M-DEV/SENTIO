"""
Audio Processing for Emotion Detection

Extracts features from audio for emotion analysis and performs speech-to-text.
"""

import logging
import numpy as np
from typing import Optional, Tuple, List
import struct

logger = logging.getLogger(__name__)


class AudioProcessor:
    """
    Processes raw audio data and extracts emotion-relevant features.
    Includes speech-to-text using faster-whisper.
    """
    
    def __init__(
        self,
        sample_rate: int = 22050,
        n_mfcc: int = 13,
        n_fft: int = 2048,
        hop_length: int = 512,
        enable_speech: bool = True,
        whisper_model: str = "base",
        whisper_device: str = "cpu",
        whisper_compute_type: str = "int8"
    ):
        """
        Initialize audio processor.
        
        Args:
            sample_rate: Audio sample rate in Hz
            n_mfcc: Number of MFCCs to extract
            n_fft: FFT window size
            hop_length: Number of samples between FFT frames
            enable_speech: Enable speech-to-text with faster-whisper
            whisper_model: Whisper model size ('tiny', 'base', 'small', 'medium', 'large')
            whisper_device: Device for whisper ('cpu' or 'cuda')
            whisper_compute_type: Compute type ('int8', 'float16', 'float32')
        """
        self.sample_rate = sample_rate
        self.n_mfcc = n_mfcc
        self.n_fft = n_fft
        self.hop_length = hop_length
        self.enable_speech = enable_speech
        
        # Try to import librosa for MFCC extraction
        try:
            import librosa
            self.librosa = librosa
            self.has_librosa = True
            logger.info('librosa available for MFCC extraction')
        except ImportError:
            logger.warning('librosa not available, using basic audio features')
            self.has_librosa = False
        
        # Initialize faster-whisper for speech-to-text
        self.whisper_model = None
        if enable_speech:
            try:
                from faster_whisper import WhisperModel
                
                self.whisper_model = WhisperModel(
                    whisper_model,
                    device=whisper_device,
                    compute_type=whisper_compute_type
                )
                logger.info(f'faster-whisper loaded: {whisper_model} on {whisper_device}')
            except ImportError:
                logger.warning('faster-whisper not available, speech-to-text disabled')
                self.enable_speech = False
            except Exception as e:
                logger.error(f'Failed to load faster-whisper: {str(e)}')
                self.enable_speech = False
    
    def process_audio_chunk(
        self,
        audio_data: bytes,
        is_float32: bool = False
    ) -> Optional[np.ndarray]:
        """
        Process a chunk of audio data.
        
        Args:
            audio_data: Raw audio bytes
            is_float32: Whether audio is in float32 format
        
        Returns:
            Feature vector or None if error
        """
        try:
            # Convert bytes to numpy array
            if is_float32:
                audio_array = np.frombuffer(audio_data, dtype=np.float32)
            else:
                audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
                audio_array /= 32768.0  # Normalize to [-1, 1]
            
            if len(audio_array) == 0:
                return None
            
            # Extract features
            features = self.extract_features(audio_array)
            return features
        
        except Exception as e:
            logger.error(f'Audio processing error: {str(e)}')
            return None
    
    def transcribe_audio(
        self,
        audio_path: str,
        beam_size: int = 5,
        language: Optional[str] = None
    ) -> Tuple[str, List[dict]]:
        """
        Transcribe audio file to text using faster-whisper.
        
        Args:
            audio_path: Path to audio file (WAV, MP3, etc.)
            beam_size: Beam size for decoding (higher = more accurate but slower)
            language: Language code (e.g., 'en', 'es') or None for auto-detect
        
        Returns:
            Tuple of (full_text, segments_list)
        """
        if not self.enable_speech or self.whisper_model is None:
            logger.warning('Speech-to-text not available')
            return "", []
        
        try:
            segments, info = self.whisper_model.transcribe(
                audio_path,
                beam_size=beam_size,
                language=language
            )
            
            # Collect all segments
            full_text = []
            segments_list = []
            
            for segment in segments:
                full_text.append(segment.text)
                segments_list.append({
                    'start': segment.start,
                    'end': segment.end,
                    'text': segment.text,
                    'confidence': getattr(segment, 'avg_logprob', 0.0)
                })
            
            transcription = " ".join(full_text)
            
            logger.info(f'Transcription complete: "{transcription[:50]}..."')
            logger.debug(f'Detected language: {info.language} (prob: {info.language_probability:.2f})')
            
            return transcription, segments_list
        
        except Exception as e:
            logger.error(f'Transcription error: {str(e)}')
            return "", []
    
    def transcribe_audio_array(
        self,
        audio_array: np.ndarray,
        beam_size: int = 5
    ) -> str:
        """
        Transcribe audio numpy array to text.
        
        Args:
            audio_array: Audio as numpy array (float32, normalized to [-1, 1])
            beam_size: Beam size for decoding
        
        Returns:
            Transcribed text
        """
        if not self.enable_speech or self.whisper_model is None:
            return ""
        
        try:
            import tempfile
            import soundfile as sf
            
            # Write temporary WAV file
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
                tmp_path = tmp_file.name
                sf.write(tmp_path, audio_array, self.sample_rate)
            
            # Transcribe
            transcription, _ = self.transcribe_audio(tmp_path, beam_size=beam_size)
            
            # Cleanup
            import os
            os.unlink(tmp_path)
            
            return transcription
        
        except Exception as e:
            logger.error(f'Array transcription error: {str(e)}')
            return ""
    
    def extract_features(self, audio: np.ndarray) -> np.ndarray:
        """
        Extract emotion-relevant features from audio.
        
        Args:
            audio: Audio signal as numpy array
        
        Returns:
            Feature vector
        """
        features = []
        
        # Extract MFCC if available
        if self.has_librosa:
            try:
                mfcc = self.librosa.feature.mfcc(
                    y=audio,
                    sr=self.sample_rate,
                    n_mfcc=self.n_mfcc,
                    n_fft=self.n_fft,
                    hop_length=self.hop_length
                )
                
                # Average over time axis
                mfcc_mean = np.mean(mfcc, axis=1)
                mfcc_std = np.std(mfcc, axis=1)
                
                features.extend(mfcc_mean)
                features.extend(mfcc_std)
            
            except Exception as e:
                logger.debug(f'MFCC extraction failed: {str(e)}')
        
        # Extract basic audio features
        basic_features = self._extract_basic_features(audio)
        features.extend(basic_features)
        
        return np.array(features, dtype=np.float32)
    
    def _extract_basic_features(self, audio: np.ndarray) -> list:
        """
        Extract basic audio features.
        
        Args:
            audio: Audio signal
        
        Returns:
            List of feature values
        """
        features = []
        
        try:
            # RMS energy
            rms = np.sqrt(np.mean(audio ** 2))
            features.append(rms)
            
            # Zero crossing rate
            zcr = np.sum(np.abs(np.diff(np.sign(audio)))) / (2 * len(audio))
            features.append(zcr)
            
            # Spectral centroid (basic approximation)
            # Use FFT to estimate
            if len(audio) > self.n_fft:
                fft = np.abs(np.fft.fft(audio[:self.n_fft]))
                freqs = np.fft.fftfreq(len(fft), 1.0 / self.sample_rate)
                spectral_centroid = np.average(freqs[:len(fft)//2], 
                                              weights=fft[:len(fft)//2])
                features.append(spectral_centroid / self.sample_rate)
            
            # Pitch (fundamental frequency) - basic autocorrelation
            if len(audio) > 2048:
                f0 = self._estimate_pitch(audio)
                features.append(f0 / self.sample_rate if f0 > 0 else 0.0)
            
            # Dynamic features
            features.append(np.max(audio))  # Max amplitude
            features.append(np.min(audio))  # Min amplitude
            features.append(np.std(audio))  # Standard deviation
        
        except Exception as e:
            logger.debug(f'Basic feature extraction error: {str(e)}')
            features.extend([0.0] * 8)  # Default values
        
        return features
    
    def _estimate_pitch(self, audio: np.ndarray) -> float:
        """
        Estimate fundamental frequency using autocorrelation.
        
        Args:
            audio: Audio signal
        
        Returns:
            Estimated frequency in Hz
        """
        try:
            # Simple autocorrelation for pitch estimation
            min_lag = int(self.sample_rate / 400)  # Max 400 Hz
            max_lag = int(self.sample_rate / 80)   # Min 80 Hz
            
            if min_lag >= len(audio) or max_lag >= len(audio):
                return 0.0
            
            # Autocorrelation
            autocorr = np.correlate(audio, audio, mode='full')
            center = len(autocorr) // 2
            autocorr = autocorr[center:center + max_lag + 1]
            
            # Find peak in lag range
            lag_range = autocorr[min_lag:max_lag]
            if len(lag_range) > 0 and np.max(lag_range) > 0:
                lag = np.argmax(lag_range) + min_lag
                pitch = self.sample_rate / lag
                return pitch
        
        except Exception as e:
            logger.debug(f'Pitch estimation error: {str(e)}')
        
        return 0.0


def parse_wav_header(wav_data: bytes) -> Tuple[int, int, int]:
    """
    Parse WAV file header to extract audio properties.
    
    Args:
        wav_data: Raw WAV file bytes
    
    Returns:
        Tuple of (sample_rate, num_channels, num_samples)
    """
    try:
        if not wav_data.startswith(b'RIFF'):
            return 0, 0, 0
        
        # Parse WAV header
        # Bytes 24-27: sample rate
        sample_rate = struct.unpack('<I', wav_data[24:28])
        
        # Byte 22-23: number of channels
        num_channels = struct.unpack('<H', wav_data[22:24])
        
        # Find 'data' chunk
        data_idx = wav_data.find(b'data')
        if data_idx == -1:
            return sample_rate, num_channels, 0
        
        # Bytes after 'data': chunk size
        chunk_size = struct.unpack('<I', wav_data[data_idx + 4:data_idx + 8])
        num_samples = chunk_size // (2 * num_channels)  # Assuming 16-bit audio
        
        return sample_rate, num_channels, num_samples
    
    except Exception as e:
        logger.debug(f'WAV header parse error: {str(e)}')
        return 0, 0, 0