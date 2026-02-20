# Test Suite

## Structure

```
test/
  run_tests.py          # Unified test runner (recommended)
  conftest.py           # pytest config + collect_ignore_glob
  unit/                 # Unit tests (no hardware, no network)
  integration/          # Integration tests (may need Ollama)
  hardware/             # Hardware tests (require robot connection)
  led_system/           # LED hardware test suite
  utils/                # Shared test helpers
```

## Running Tests

```bash
# All unit tests (recommended)
python3 test/run_tests.py --type unit

# All tests
python3 test/run_tests.py

# Specific type
python3 test/run_tests.py --type hardware
python3 test/run_tests.py --type integration

# Via pytest
pytest test/unit/ -v
```

## Unit Tests

| Test File | Coverage |
|-----------|----------|
| `test_action_registry.py` | Action definitions, API codes, template responses |
| `test_safety_compiler.py` | Battery gating, standing prerequisites, whitelist |
| `test_safety_regression.py` | Safety edge cases and regression scenarios |
| `test_channel_router.py` | Dual/legacy/shadow routing logic |
| `test_voice_commander.py` | VoiceCommander lifecycle and signal handling |
| `test_asr_bridge.py` | ASR result filtering, dedup, command dispatch |
| `test_wake_word.py` | Wake word matching, false positive rejection |
| `test_audio_capture.py` | Audio capture device detection |
| `test_pcm_utils.py` | PCM resampling correctness |
| `test_audit_extension.py` | Audit log field validation |
| `test_commander_wakeup.py` | Wakeup animation safety gates |
| `test_commander_migration.py` | API migration compatibility |
| `test_sdk_state_provider.py` | SDK state query mocking |
| `test_sanitize_response.py` | Japanese output validation |
| `test_safety_integration.py` | Safety pipeline integration |

## Notes

- System pytest is 4.6.9 (ROS2 Foxy constraint)
- `pytest.ini` required (pyproject.toml `[tool.pytest.ini_options]` ignored by 4.6.9)
- ROS2 plugins auto-disabled via `addopts` in `pytest.ini`
