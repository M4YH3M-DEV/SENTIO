"""
Audio File Manager for SENTIO TTS

Handles audio file I/O, caching, and playback coordination.
"""

import os
import json
import logging
from pathlib import Path
from typing import Optional, Dict
from datetime import datetime
import hashlib


logger = logging.getLogger(__name__)


class AudioManager:
    """
    Manages audio file lifecycle for TTS synthesis.
    
    - Save synthesized audio to disk
    - Organize files by timestamp/hash
    - Implement caching to avoid re-synthesis
    - Clean up old files
    """
    
    def __init__(self, base_dir: str = '/tmp/sentio_tts'):
        """
        Initialize audio manager.
        
        Args:
            base_dir: Base directory for audio storage
        """
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)
        
        # Create subdirectories
        self.audio_dir = self.base_dir / 'audio'
        self.cache_dir = self.base_dir / 'cache'
        self.logs_dir = self.base_dir / 'logs'
        
        for d in [self.audio_dir, self.cache_dir, self.logs_dir]:
            d.mkdir(parents=True, exist_ok=True)
        
        self.cache: Dict[str, str] = {}  # text_hash -> filepath
        
        logger.info(f'AudioManager initialized | Base: {self.base_dir}')
    
    def text_to_hash(self, text: str, voice: str) -> str:
        """
        Generate hash for text+voice combination for caching.
        
        Args:
            text: Input text
            voice: Voice identifier
        
        Returns:
            SHA256 hash string
        """
        combined = f'{text}::{voice}'.encode('utf-8')
        return hashlib.sha256(combined).hexdigest()[:16]
    
    def get_cached_audio(self, text: str, voice: str) -> Optional[str]:
        """
        Check if audio for this text/voice combo is cached.
        
        Args:
            text: Input text
            voice: Voice identifier
        
        Returns:
            Path to cached audio file, or None if not cached
        """
        key = self.text_to_hash(text, voice)
        
        if key in self.cache:
            path = Path(self.cache[key])
            if path.exists():
                logger.debug(f'Cache hit: {key}')
                return str(path)
            else:
                del self.cache[key]
        
        return None
    
    def save_audio(
        self,
        audio_data: bytes,
        text: str,
        voice: str,
        format: str = 'wav'
    ) -> Optional[str]:
        """
        Save synthesized audio to disk.
        
        Args:
            audio_data: Raw audio bytes
            text: Original text
            voice: Voice identifier
            format: File format (default: 'wav')
        
        Returns:
            Path to saved file, or None if failed
        """
        if not audio_data:
            logger.error('No audio data to save')
            return None
        
        try:
            # Generate filename
            text_hash = self.text_to_hash(text, voice)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'{text_hash}_{timestamp}.{format}'
            filepath = self.audio_dir / filename
            
            # Write audio file
            with open(filepath, 'wb') as f:
                f.write(audio_data)
            
            # Update cache
            self.cache[text_hash] = str(filepath)
            
            logger.info(f'Audio saved: {filepath} ({len(audio_data)} bytes)')
            
            # Save metadata
            self._save_metadata(filepath, text, voice, len(audio_data))
            
            return str(filepath)
        
        except Exception as e:
            logger.error(f'Failed to save audio: {str(e)}')
            return None
    
    def _save_metadata(
        self,
        filepath: str,
        text: str,
        voice: str,
        size_bytes: int
    ):
        """
        Save metadata for audio file.
        
        Args:
            filepath: Path to audio file
            text: Original text
            voice: Voice identifier
            size_bytes: File size in bytes
        """
        try:
            metadata = {
                'timestamp': datetime.now().isoformat(),
                'text': text[:100],  # Truncate for storage
                'voice': voice,
                'size_bytes': size_bytes,
                'filepath': filepath,
            }
            
            meta_path = Path(filepath).with_suffix('.json')
            with open(meta_path, 'w') as f:
                json.dump(metadata, f, indent=2)
        
        except Exception as e:
            logger.debug(f'Metadata save error: {str(e)}')
    
    def cleanup_old_files(self, max_age_hours: int = 24) -> int:
        """
        Delete audio files older than specified age.
        
        Args:
            max_age_hours: Maximum age in hours
        
        Returns:
            Number of files deleted
        """
        try:
            import time
            current_time = time.time()
            max_age_seconds = max_age_hours * 3600
            deleted_count = 0
            
            for audio_file in self.audio_dir.glob('*.wav'):
                file_age = current_time - audio_file.stat().st_mtime
                
                if file_age > max_age_seconds:
                    audio_file.unlink()
                    
                    # Also delete metadata
                    meta_file = audio_file.with_suffix('.json')
                    if meta_file.exists():
                        meta_file.unlink()
                    
                    deleted_count += 1
            
            if deleted_count > 0:
                logger.info(f'Cleaned up {deleted_count} old audio files')
            
            return deleted_count
        
        except Exception as e:
            logger.error(f'Cleanup error: {str(e)}')
            return 0
    
    def get_disk_usage(self) -> Dict[str, int]:
        """
        Get disk usage statistics.
        
        Returns:
            Dictionary with size information
        """
        try:
            total_size = sum(
                f.stat().st_size for f in self.audio_dir.rglob('*')
                if f.is_file()
            )
            
            file_count = len(list(self.audio_dir.glob('*.wav')))
            
            return {
                'total_bytes': total_size,
                'total_mb': round(total_size / (1024 * 1024), 2),
                'file_count': file_count,
            }
        
        except Exception as e:
            logger.error(f'Disk usage calculation error: {str(e)}')
            return {'total_bytes': 0, 'total_mb': 0.0, 'file_count': 0}
    
    def get_audio_path(self, text: str, voice: str) -> Optional[str]:
        """
        Get path to audio file (cached or None).
        
        Args:
            text: Input text
            voice: Voice identifier
        
        Returns:
            Path string or None
        """
        return self.get_cached_audio(text, voice)
