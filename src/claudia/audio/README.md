# Audio Module

Voice pipeline: USB microphone capture, ASR transcription, wake word detection.

## Modules

| File | Role |
|------|------|
| `audio_capture.py` | USB mic → arecord → 44.1kHz→16kHz resample → UDS socket |
| `asr_bridge.py` | ASR result consumer: dedup, filter, wake word gate → Brain |
| `wake_word.py` | Wake word matcher (exact prefix) + listening window gate |
| `pcm_utils.py` | PCM resampling utility (numpy index-based, 44100→16000) |
| `asr_service/` | ASR server subprocess (faster-whisper + silero-vad) |

## Voice Pipeline

```
USB Mic (AT2020USB-XP, 44100Hz)
  │ arecord → resample → 16kHz frames
  v
AudioCapture ──→ audio.sock ──→ ASR Server (subprocess)
                                  ├── silero-vad + emergency detection
                                  ├── faster-whisper base (CPU int8)
                                  v
ASRBridge ←── result.sock ←───── JSON Lines
  ├── emergency → bypass wake word → brain
  ├── transcript → confidence filter → wake word gate → brain
  └── command worker → brain.process_and_execute()
```

## Wake Word

When `CLAUDIA_WAKE_WORD_ENABLED=1`:
- Exact prefix matching against `WAKE_PREFIXES` list only
- Inline: "クラちゃん踊って" → strips prefix → "踊って" to brain
- Standalone: "クラちゃん" → opens 5s listening window → next utterance passes
- Emergency commands always bypass the wake word gate
