"""
Audio Processing for Emotion Detection

Extracts features from audio for emotion analysis.
"""

import logging
import numpy as np
from typing import Optional, Tuple
import struct


logger = logging.getLogger(__name__)


class AudioProcessor:
    """
    Processes raw audio data and extracts emotion-relevant features.
    """
    
    def __init__(
        self,
        sample_rate: int = 22050,
        n_mfcc: int = 13,
        n_fft: int = 2048,
        hop_length: int = 512
    ):
        """
        Initialize audio processor.
        
        Args:
            sample_rate: Audio sample rate in Hz
            n_mfcc: Number of MFCCs to extract
            n_fft: FFT window size
            hop_length: Number of samples between FFT frames
        """
        self.sample_rate = sample_rate
        self.n_mfcc = n_mfcc
        self.n_fft = n_fft
        self.hop_length = hop_length
        
        # Try to import librosa for MFCC extraction
        try:
            import librosa
            self.librosa = librosa
            self.has_librosa = True
            logger.info('librosa available for MFCC extraction')
        except ImportError:
            logger.warning('librosa not available, using basic audio features')
            self.has_librosa = False
    
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
        sample_rate = struct.unpack('<I', wav_data[24:28])[0]
        
        # Byte 22-23: number of channels
        num_channels = struct.unpack('<H', wav_data[22:24])[0]
        
        # Find 'data' chunk
        data_idx = wav_data.find(b'data')
        if data_idx == -1:
            return sample_rate, num_channels, 0
        
        # Bytes after 'data': chunk size
        chunk_size = struct.unpack('<I', wav_data[data_idx + 4:data_idx + 8])[0]
        num_samples = chunk_size // (2 * num_channels)  # Assuming 16-bit audio
        
        return sample_rate, num_channels, num_samples
    
    except Exception as e:
        logger.debug(f'WAV header parse error: {str(e)}')
        return 0, 0, 0
