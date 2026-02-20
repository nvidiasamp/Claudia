# Ollama Modelfiles

Ollama Modelfile definitions for creating the Claudia LLM models.

## Active Models

| File | Ollama Name | Role |
|------|-------------|------|
| `ClaudiaIntelligent_7B_v2.0` | `claudia-7b:v2.0` | Brain model (full response + action code) |
| `ClaudiaAction_v3.0` | `claudia-action-v3` | Action-only model (~30 tokens, `{"a":N}`) |

## Usage

```bash
# Create models from Modelfiles
ollama create claudia-7b:v2.0 -f models/ClaudiaIntelligent_7B_v2.0
ollama create claudia-action-v3 -f models/ClaudiaAction_v3.0

# Verify
ollama list | grep claudia
```

## Notes

- Both models are based on `qwen2.5:7b`
- Action model uses `num_predict=30`, `temperature=0.0` for deterministic short output
- Brain model uses `temperature=0.0` with JSON mode
- `start_production_brain.sh` automatically creates missing models on launch
