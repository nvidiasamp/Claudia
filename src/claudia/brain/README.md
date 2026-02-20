# Brain Module

Core command processing pipeline for the Claudia robot.

## Modules

| File | Role |
|------|------|
| `production_brain.py` | Main pipeline: emergency → cache → conversation → LLM → safety → execute |
| `channel_router.py` | Dual-channel LLM router (legacy/dual/shadow modes) |
| `action_registry.py` | Single source of truth: 18 actions, API codes, template responses |
| `safety_compiler.py` | Unified safety gate: whitelist, battery, standing prerequisites |
| `audit_logger.py` | Structured JSON audit trail to `logs/audit/` |
| `audit_routes.py` | Canonical route name constants for audit logging |
| `mock_sport_client.py` | SportClient simulator for testing without hardware |
| `sdk_state_provider.py` | Direct SDK state queries (battery, posture) |

## Pipeline Flow

```
Input → Emergency Bypass → Hot Cache → Conversation Detection
  → LLM Inference (via ChannelRouter) → SafetyCompiler.compile()
  → execute_action() → BrainOutput
```

## Key Design Decisions

- `process_and_execute()` is the atomic entry point (process + execute + status)
- `SafetyCompiler` is never bypassed — all paths go through `compile()`
- `ChannelRouter` is decision-only — never executes or touches safety
- Ollama JSON mode with `temperature=0.0` for deterministic output
