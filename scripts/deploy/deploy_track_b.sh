#!/bin/bash
# Track B Deployment Script: Pull/create new model and quick test

set -e

cd $HOME/claudia

echo "=================================="
echo "Track B: New Model Deployment"
echo "=================================="
echo ""

# 1. Ensure base model exists
echo "Checking base model..."
if ! ollama list | grep -q "qwen2.5:7b"; then
    echo "Pulling qwen2.5:7b (approx. 4.7GB, INT4 quantized)..."
    ollama pull qwen2.5:7b
else
    echo "   qwen2.5:7b already exists"
fi

# 2. Create new Modelfile model
echo ""
echo "Creating claudia-intelligent-7b:v1..."
if ollama list | grep -q "claudia-intelligent-7b:v1"; then
    echo "   Model already exists, removing old version..."
    ollama rm claudia-intelligent-7b:v1 2>/dev/null || true
fi

ollama create claudia-intelligent-7b:v1 -f ClaudiaIntelligent_Qwen7B

echo "   Model creation complete"

# 3. Quick test (fix: use Python library instead of CLI --format json)
echo ""
echo "Quick testing new model..."
echo ""

python3 - <<'PYEOF'
import ollama
import json

# Japanese robot commands used as test inputs
test_commands = [
    "座って",    # sit
    "立って",    # stand up
    "可愛い",    # cute
    "stop"
]

print("Testing with Python ollama library (format='json')...\n")
for cmd in test_commands:
    print(f"Test: {cmd}")
    try:
        response = ollama.chat(
            model="claudia-intelligent-7b:v1",
            messages=[{'role': 'user', 'content': cmd}],
            format='json',
            options={'temperature': 0.1, 'num_predict': 30}
        )
        content = response['message']['content']
        data = json.loads(content)
        api_code = data.get('api_code') or data.get('a')
        print(f"  -> api_code: {api_code}")
        print(f"  -> raw: {content[:80]}...")
    except Exception as e:
        print(f"  Error: {e}")
    print("")
PYEOF

echo "=================================="
echo "Track B deployment complete!"
echo ""
echo "Next steps:"
echo "1. Run full evaluation: python3 test/test_model_comparison.py"
echo "2. View comparison results: test/model_comparison_results.json"
echo "=================================="
