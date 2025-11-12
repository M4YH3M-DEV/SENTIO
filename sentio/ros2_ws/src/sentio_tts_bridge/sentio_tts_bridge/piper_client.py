"""
Piper TTS HTTP Client

Communicates with Piper TTS HTTP server for text synthesis.
"""

import requests
import logging
import json
from typing import Optional, Tuple
from datetime import datetime
import time


logger = logging.getLogger(__name__)


class PiperClientError(Exception):
    """Base exception for Piper client errors."""
    pass


class PiperConnectionError(PiperClientError):
    """Raised when unable to connect to Piper server."""
    pass


class PiperSynthesisError(PiperClientError):
    """Raised when synthesis fails."""
    pass


class PiperClient:
    """
    HTTP client for Piper TTS server.
    
    Handles communication with Piper, text encoding, and audio retrieval.
    """
    
    def __init__(
        self,
        server_url: str = "http://localhost:5000",
        timeout: float = 30.0,
        max_retries: int = 3,
        retry_delay_s: float = 1.0
    ):
        """
        Initialize Piper client.
        
        Args:
            server_url: Base URL of Piper HTTP server
            timeout: Request timeout in seconds
            max_retries: Maximum retry attempts
            retry_delay_s: Delay between retries
        """
        self.server_url = server_url.rstrip('/')
        self.timeout = timeout
        self.max_retries = max_retries
        self.retry_delay_s = retry_delay_s
        self.session = requests.Session()
        self.connected = False
        
        logger.info(f'PiperClient initialized | URL: {self.server_url}')
    
    def health_check(self) -> bool:
        """
        Check if Piper server is online and responsive.
        
        Returns:
            True if server is healthy, False otherwise
        """
        try:
            response = self.session.get(
                f'{self.server_url}/api/voices',
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                self.connected = True
                logger.info('Piper server health check: OK')
                return True
            else:
                logger.warning(f'Piper health check failed: {response.status_code}')
                self.connected = False
                return False
        
        except Exception as e:
            logger.error(f'Piper health check error: {str(e)}')
            self.connected = False
            return False
    
    def get_voices(self) -> Optional[dict]:
        """
        Get list of available voices from Piper server.
        
        Returns:
            Dictionary of voices or None if failed
        """
        try:
            response = self.session.get(
                f'{self.server_url}/api/voices',
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                logger.error(f'Failed to get voices: {response.status_code}')
                return None
        
        except Exception as e:
            logger.error(f'Get voices error: {str(e)}')
            return None
    
    def synthesize(
        self,
        text: str,
        voice: str = "default",
        **kwargs
    ) -> Tuple[bool, Optional[bytes], Optional[str]]:
        """
        Synthesize text to speech using Piper.
        
        Args:
            text: Text to synthesize
            voice: Voice profile name
            **kwargs: Additional synthesis parameters
        
        Returns:
            Tuple of (success, audio_bytes, error_message)
        """
        if not text or not text.strip():
            logger.warning('Empty text provided for synthesis')
            return False, None, 'Empty text'
        
        # Limit text length
        max_length = 500
        if len(text) > max_length:
            logger.warning(f'Text truncated from {len(text)} to {max_length} chars')
            text = text[:max_length]
        
        payload = {
            'text': text,
            'speaker': voice,
            **kwargs
        }
        
        for attempt in range(self.max_retries):
            try:
                logger.debug(f'Synthesis attempt {attempt + 1}/{self.max_retries}')
                
                response = self.session.post(
                    f'{self.server_url}/api/synthesize',
                    json=payload,
                    timeout=self.timeout,
                    stream=True
                )
                
                if response.status_code == 200:
                    # Read audio data
                    audio_data = b''
                    for chunk in response.iter_content(chunk_size=8192):
                        if chunk:
                            audio_data += chunk
                    
                    if audio_data:
                        logger.info(
                            f'Synthesis successful: {len(text)} chars -> '
                            f'{len(audio_data)} bytes'
                        )
                        return True, audio_data, None
                    else:
                        msg = 'No audio data received'
                        logger.error(msg)
                        return False, None, msg
                
                elif response.status_code == 400:
                    msg = f'Bad request: {response.text}'
                    logger.error(msg)
                    return False, None, msg
                
                elif response.status_code == 503:
                    msg = 'Piper server busy'
                    logger.warning(msg)
                    if attempt < self.max_retries - 1:
                        time.sleep(self.retry_delay_s)
                        continue
                    return False, None, msg
                
                else:
                    msg = f'HTTP {response.status_code}: {response.text}'
                    logger.error(msg)
                    return False, None, msg
            
            except requests.Timeout:
                msg = f'Request timeout after {self.timeout}s'
                logger.error(msg)
                if attempt < self.max_retries - 1:
                    time.sleep(self.retry_delay_s)
                    continue
                return False, None, msg
            
            except Exception as e:
                msg = f'Synthesis error: {str(e)}'
                logger.error(msg)
                if attempt < self.max_retries - 1:
                    time.sleep(self.retry_delay_s)
                    continue
                return False, None, msg
        
        return False, None, 'Max retries exceeded'
    
    def close(self):
        """Close HTTP session."""
        if self.session:
            self.session.close()
            logger.info('PiperClient session closed')
