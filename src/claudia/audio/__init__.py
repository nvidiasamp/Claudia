# src/claudia/audio/__init__.py
"""
Audio Processing Module

- pcm_utils: PCM resampling utility
- audio_capture: USB microphone capture (arecord -> UDS)
- asr_bridge: ASR result consumer -> ProductionBrain bridge
- asr_service/: ASR server (faster-whisper + VAD + 3-way UDS)
"""

from .pcm_utils import resample_pcm_int16
from .audio_capture import AudioCapture
from .asr_bridge import ASRBridge

__all__ = [
    "resample_pcm_int16",
    "AudioCapture",
    "ASRBridge",
]
