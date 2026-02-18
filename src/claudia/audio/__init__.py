# src/claudia/audio/__init__.py
"""
音声処理モジュール

- pcm_utils: PCM 重采样ユーティリティ
- audio_capture: USB マイクキャプチャ (arecord → UDS)
- asr_bridge: ASR 結果消費 → ProductionBrain 橋渡し
- asr_service/: ASR サーバー (faster-whisper + VAD + 3路 UDS)
"""

from .pcm_utils import resample_pcm_int16
from .audio_capture import AudioCapture
from .asr_bridge import ASRBridge

__all__ = [
    "resample_pcm_int16",
    "AudioCapture",
    "ASRBridge",
]
