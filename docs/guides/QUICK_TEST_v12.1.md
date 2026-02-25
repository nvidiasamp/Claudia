# v12.1-simple Quick Test Guide

**Purpose**: Verify edge case fixes ("godee"/"pong" issues resolved)

---

## Prerequisite Check

```bash
# 1. Confirm v12.1-simple model has been created
ollama list | grep v12.1-simple
# Expected output: claudia-go2-7b:v12.1-simple

# 2. If it doesn't exist, create the model
ollama create claudia-go2-7b:v12.1-simple -f models/ClaudiaIntelligent_7B_v2.0

# 3. Confirm ProductionBrain configuration
grep "v12.1-simple" src/claudia/brain/production_brain.py
# Expected output: self.model_7b = os.getenv("BRAIN_MODEL_7B", "claudia-go2-7b:v12.1-simple")
```

---

## Test 1: Ollama Direct Test (Recommended)

**Advantage**: Directly tests model output without other interference

```bash
# Edge cases (previously failed, should now be fixed)
echo "今日はいい天気ですね" | ollama run claudia-go2-7b:v12.1-simple
# Expected: {"r":"そうですね","a":null}
# v12-simple output was: {"r":" godee"}

echo "ちんちん" | ollama run claudia-go2-7b:v12.1-simple
# Expected: {"r":"お辞儀します","a":1016}
# v12-simple output was: {"r":" pong"}

echo "おはよう" | ollama run claudia-go2-7b:v12.1-simple
# Expected: {"r":"おはようございます","a":null}

# Core functionality (ensure no regression)
echo "可愛いね" | ollama run claudia-go2-7b:v12.1-simple
# Expected: {"r":"ありがとうございます","a":1036}

echo "立ってそしてダンス" | ollama run claudia-go2-7b:v12.1-simple
# Expected: {"r":"立ってからダンスします","s":[1004,1023]}

echo "疲れた" | ollama run claudia-go2-7b:v12.1-simple
# Expected: {"r":"休みますね","a":1009}

echo "座って" | ollama run claudia-go2-7b:v12.1-simple
# Expected: {"r":"座ります","a":1009}
```

---

## Test 2: ProductionBrain Interactive Test

**Advantage**: Tests the complete system including code-level protections

```bash
# Start production commander
./start_production_brain.sh
# Select: 1) Simulation mode (safe testing)

# Enter the following commands in the interactive interface:
```

### Edge Case Tests

```
Claudia> 今日はいい天気ですね
Expected: Response contains Japanese, no "godee"
Expected: api_code: null (casual chat, no action)

Claudia> ちんちん
Expected: Response contains Japanese, no "pong"
Expected: api_code: 1016 (bow) or null

Claudia> おはよう
Expected: Response: "おはようございます" or similar
Expected: api_code: null

Claudia> よくわからない言葉xyzabc
Expected: Response: "すみません、よく分かりません"
Expected: api_code: null
```

### Core Functionality Tests

```
Claudia> 可愛いね
Expected: Heart action (1036)

Claudia> 立ってそしてダンス
Expected: Sequence [1004, 1023]

Claudia> 疲れた
Expected: Sit (1009)

Claudia> 座って
Expected: Sit (1009)
```

---

## Test 3: Code-Level Unit Tests

**Advantage**: Verifies `_sanitize_response()` function logic

```bash
python3 << 'EOF'
from src.claudia.brain.production_brain import ProductionBrain

brain = ProductionBrain(use_real_hardware=False)

# Test that invalid input returns default response
test_cases = [
    (' godee', 'すみません、よく分かりません'),
    (' pong', 'すみません、よく分かりません'),
    ('', 'すみません、よく分かりません'),
    ('   ', 'すみません、よく分かりません'),
    ('ok', 'すみません、よく分かりません'),
    ('hello', 'すみません、よく分かりません'),
]

print("Testing _sanitize_response() function:")
all_pass = True
for input_str, expected in test_cases:
    result = brain._sanitize_response(input_str)
    status = "PASS" if result == expected else "FAIL"
    print(f"{status} '{input_str}' -> '{result}'")
    all_pass = all_pass and (result == expected)

# Test that valid Japanese is preserved
valid_cases = [
    'ありがとうございます',
    'こんにちは',
    '座ります',
    'そうですね',
]

for input_str in valid_cases:
    result = brain._sanitize_response(input_str)
    status = "PASS" if result == input_str else "FAIL"
    print(f"{status} '{input_str}' -> '{result}' (should be preserved)")
    all_pass = all_pass and (result == input_str)

if all_pass:
    print("\nAll unit tests passed!")
else:
    print("\nSome tests failed")
EOF
```

---

## Test 4: Audit Log Inspection

**Advantage**: Monitors output quality in actual usage

```bash
# After using for a while, check audit logs
tail -100 logs/$(date '+%Y%m')/audit_*.jsonl | jq '.llm_output.response'

# Check for nonsense output
grep -E 'godee|pong' logs/$(date '+%Y%m')/audit_*.jsonl
# Expected: No results (godee/pong not found)

# Count non-Japanese responses (should only be "すみません、よく分かりません")
tail -100 logs/$(date '+%Y%m')/audit_*.jsonl | \
  jq -r '.llm_output.response' | \
  grep -v '[\u3040-\u309f\u30a0-\u30ff\u4e00-\u9faf]'
# Expected: No results or only a few default responses
```

---

## Expected Output Comparison

### Edge Case Fix Comparison

| Input | v12-simple (Before) | v12.1-simple (After) |
|-------|---------------------|----------------------|
| "今日はいい天気ですね" | `{"r":" godee"}` | `{"r":"そうですね","a":null}` |
| "ちんちん" | `{"r":" pong"}` | `{"r":"お辞儀します","a":1016}` |
| "おはよう" | Unknown | `{"r":"おはようございます","a":null}` |
| Nonsense input | Potentially anomalous | `{"r":"すみません、よく分かりません","a":null}` |

### Core Functionality Unchanged

| Input | Expected Output | v12.1-simple |
|-------|----------------|-------------|
| "可愛いね" | `{"r":"ありがとうございます","a":1036}` | Maintained |
| "立ってそしてダンス" | `{"r":"立ってからダンスします","s":[1004,1023]}` | Maintained |
| "疲れた" | `{"r":"休みますね","a":1009}` | Maintained |
| "座って" | `{"r":"座ります","a":1009}` | Maintained |

---

## Troubleshooting

### Issue 1: Model does not exist

```bash
# Symptom
$ ollama run claudia-go2-7b:v12.1-simple
Error: model not found

# Solution
$ ollama create claudia-go2-7b:v12.1-simple -f models/ClaudiaIntelligent_7B_v2.0
```

### Issue 2: Still outputs "godee"/"pong"

```bash
# Symptom
$ echo "今日はいい天気ですね" | ollama run claudia-go2-7b:v12.1-simple
{"r":" godee"}

# Diagnosis
1. Confirm using v12.1-simple model, not v12-simple
   ollama list | grep claudia

2. Confirm Modelfile has been updated
   grep "Rule 6" models/ClaudiaIntelligent_7B_v2.0
   grep "今日はいい天気ですね" models/ClaudiaIntelligent_7B_v2.0

3. Recreate the model
   ollama rm claudia-go2-7b:v12.1-simple
   ollama create claudia-go2-7b:v12.1-simple -f models/ClaudiaIntelligent_7B_v2.0
```

### Issue 3: ProductionBrain not using v12.1

```bash
# Symptom
Log shows: 7B model: claudia-go2-7b:v12-simple

# Diagnosis
grep "model_7b" src/claudia/brain/production_brain.py | head -5
# Should show: self.model_7b = os.getenv("BRAIN_MODEL_7B", "claudia-go2-7b:v12.1-simple")

# If it shows v12-simple, manually update:
# Edit src/claudia/brain/production_brain.py Line 80
# Or set environment variable:
export BRAIN_MODEL_7B=claudia-go2-7b:v12.1-simple
./start_production_brain.sh
```

---

## Success Criteria

**Test 1 Passed**: Ollama direct test outputs Japanese for all cases, no "godee"/"pong"

**Test 2 Passed**: ProductionBrain interactive test passes edge cases and core functionality

**Test 3 Passed**: `_sanitize_response()` unit tests all pass

**Test 4 Passed**: Audit logs contain no anomalous output

---

## Follow-up Actions

After tests pass:

1. **Mark v12.1 as production version**
   ```bash
   ollama tag claudia-go2-7b:v12.1-simple claudia-go2-7b:production
   ```

2. **Delete old version (optional)**
   ```bash
   ollama rm claudia-go2-7b:v12-simple
   ```

3. **Update documentation**
   - Mark v12.1-simple as recommended version in README.md

4. **Continuous monitoring**
   - Check audit logs weekly
   - Collect new edge cases
   - Gradually expand few-shot examples

---

**Author**: Claude Code
**Created**: 2025-11-18
**Purpose**: v12.1-simple edge case fix verification
