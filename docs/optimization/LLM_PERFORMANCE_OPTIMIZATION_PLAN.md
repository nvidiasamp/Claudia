# Claudia LLM Performance Optimization Plan

**Date**: 2025-11-14
**Version**: v1.0
**Problem Source**: 3 Critical issues found during hardware testing

---

## Problem Summary

### 1. Severe LLM Performance Bottleneck - Critical

**Symptoms**:
```
3B model response (2866ms)  # 2.8s delay
3B model response (3600ms)  # 3.6s delay
Model timeout(5s): claudia-go2-3b:v11.2  # Timeout
```

**Impact**:
- Real-time robot control delay is unacceptable (should be <500ms)
- Extremely poor user experience (3-second wait)
- System unusable when hot path hit rate is low

**Root Causes**:
1. **3B model too small**, insufficient understanding causes multiple retries
2. **Ollama configuration not optimized** (batch_size, ctx_size may be insufficient)
3. **Low Jetson GPU utilization** (GR3D_FREQ shows 0%, may be monitoring delay)
4. **Local inference architecture limitation** (Qwen2.5-3B still has 3B parameters even after quantization)

---

### 2. LLM Output Language Confusion - High

**Symptoms**:
```
Response: 立ちます然后は挨拶します  # Mixed Japanese + Chinese
Response: 前扑します  # "扑" is a Chinese character
```

**Impact**:
- TTS playback may fail (Japanese TTS cannot read Chinese)
- Unprofessional user experience
- Multilingual confusion makes intelligence appear low

**Root Causes**:
```bash
# Current Modelfile SYSTEM prompt
SYSTEM Claudia dog. Reply one JSON.
pounce→{"r":"前扑します","a":1032}  # "前扑" is Chinese
scrape→{"r":"擦ります","a":1029}    # Correct Japanese
```

1. **Modelfile training data mixes Chinese and Japanese**
2. **SYSTEM prompt doesn't explicitly constrain "must be pure Japanese"**
3. **Examples themselves contain errors** ("前扑します")

---

### 3. Insufficient Intelligent Understanding - High

**Symptoms**:
```
くら> 立ってそして挨拶して  # "Stand then greet"
Response: 立ちます然后は挨拶します
API: 1018  # Non-existent API (correct should be sequence [1004,1016])
Execution failed
```

```
くら> 可愛いいね  # "So cute"
Response: ありがとうございます！  # Only replies, no action
# User may expect a cute action (e.g., Heart)
```

**Impact**:
- Complex sequence understanding failure
- Cannot correctly decompose multi-step tasks
- Dialog vs action intent judgment not intelligent enough

**Root Causes**:
1. **3B model capacity too small**, cannot understand complex semantics
2. **Training data lacks sequence samples** ("立ってそして挨拶" -> [1004,1016])
3. **Dialog detection rules too simple** ("可愛いね" may need action + reply)

---

## Optimization Plan

### P0 - Emergency Fix (Complete Today)

#### 1.1 Fix Modelfile Language Confusion

**Operation**:
```bash
# Create new pure Japanese Modelfile
cat > ClaudiaGo2_v11.3_Japanese <<'EOF'
FROM claudia-go2-3b:v11.2

SYSTEM """あなたはClaudiaという四足ロボット犬のAIです。
指示を理解し、JSON形式で日本語の返事と動作APIコードを返してください。
必ず日本語のみを使用し、中国語や英語を混ぜないでください。

動作マッピング:
- 座って/すわって/おすわり → {"r":"座ります","a":1009}
- 立って/たって → {"r":"立ちます","a":1004}
- 伏せて/横になって → {"r":"伏せます","a":1005}
- 止まって/ストップ → {"r":"止まります","a":1003}
- こんにちは/ハロー → {"r":"こんにちは","a":1016}
- 伸びして → {"r":"伸びをします","a":1017}
- ハート/可愛い動作/いい子 → {"r":"ハートします","a":1036}
- 踊って/ダンス → {"r":"踊ります","a":1023}

複数動作の場合は"seq"配列を使用:
例: 座ってから挨拶 → {"r":"座ってから挨拶します","a":null,"seq":[1009,1016]}
"""

PARAMETER num_predict 50
PARAMETER temperature 0.2
PARAMETER top_p 0.8
PARAMETER num_ctx 2048
EOF

# Create new model
ollama create claudia-go2-3b:v11.3 -f ClaudiaGo2_v11.3_Japanese
```

**Verification**:
```bash
echo "立ってそして挨拶して" | ollama run claudia-go2-3b:v11.3
# Expected: {"r":"立ってから挨拶します","a":null,"seq":[1004,1016]}
# Should not contain Chinese characters
```

**Expected improvement**: Eliminates Chinese-Japanese mixing, but performance still slow (needs P1 solution)

---

#### 1.2 Expand Hot Path Coverage (Reduce LLM Calls)

**Operation**: Modify `production_brain.py`
```python
# Current hot path only has 13 keywords
HOTPATH_MAP = {
    '座って': 1009, 'すわって': 1009, '座る': 1009,
    # ... 13 entries
}

# Expand to 50+ variants
HOTPATH_MAP = {
    # Sit variants
    '座って': 1009, 'すわって': 1009, '座る': 1009,
    'おすわり': 1009, 'すわり': 1009, 'お座り': 1009,
    'sit': 1009, 'sit down': 1009, '坐下': 1009,

    # Stand variants
    '立って': 1004, 'たって': 1004, '立つ': 1004,
    'お立ち': 1004, '起きて': 1004,
    'stand': 1004, 'stand up': 1004, '站立': 1004,

    # Greeting variants
    'こんにちは': 1016, 'ハロー': 1016, 'ハイ': 1016,
    'やあ': 1016, 'おはよう': 1016,
    'hello': 1016, 'hi': 1016, '你好': 1016,

    # Cute action variants
    'ハート': 1036, 'はーと': 1036, 'いい子': 1036,
    '可愛い': 1036, 'かわいい': 1036,
    'heart': 1036, 'cute': 1036, '爱心': 1036,

    # ... more variants
}
```

**Expected improvement**: Hot path hit rate from 20% -> 80%, most commands <1ms response

---

#### 1.3 Fix API 1018 Error (Add Predefined Sequences)

**Operation**: Add common sequences to hot path
```python
# Add sequence hot path in process_command
SEQUENCE_HOTPATH = {
    '立ってから挨拶': [1004, 1016],
    '立って挨拶': [1004, 1016],
    '立ってそして挨拶': [1004, 1016],
    '座ってから挨拶': [1009, 1016],
    '座って挨拶': [1009, 1016],
}

# Add sequence check after hot path check
for key, seq in SEQUENCE_HOTPATH.items():
    if key in command:
        return BrainOutput(
            response="了解しました",
            sequence=seq,
            confidence=1.0
        )
```

**Expected improvement**: Common sequence commands no longer call LLM, avoiding 1018 error

---

### P1 - Performance Optimization (Complete This Week)

#### 2.1 Upgrade Main Model to 7B

**Current situation**:
- 3B: Fast but poor understanding -> causes errors needing retry -> actually slower
- 7B: Better understanding but slow -> one success may be faster than multiple 3B retries

**Operation**:
```python
# Modify production_brain.py default routing strategy
# Current: Simple commands -> 3B, Complex commands -> 7B
# Optimized: All non-hot-path -> 7B (avoid 3B understanding errors)

# Line 968-976
if len(command) > 20 or "そして" in command or "から" in command:
    # Complex commands -> 7B
    selected_7b = self.model_7b_pool[0]
    enhanced_cmd = self._build_enhanced_prompt(command, selected_7b, state_snapshot)
    result = await self._call_ollama_v2(selected_7b, enhanced_cmd, timeout=10)
else:
    # Changed to: All commands use 7B (3B understanding insufficient)
    selected_7b = self.model_7b_pool[0]
    enhanced_cmd = self._build_enhanced_prompt(command, selected_7b, state_snapshot)
    result = await self._call_ollama_v2(selected_7b, enhanced_cmd, timeout=8)
```

**A/B Testing**:
```bash
# Test samples
commands = [
    "立ってそして挨拶して",  # Complex sequence
    "可愛い動作して",        # Semantic understanding
    "疲れた",               # Metaphor
]

# Comparison
python3 test/test_3b_vs_7b_performance.py
```

**Expected improvement**:
- Accuracy: 60% -> 90%
- Average latency: 3000ms (3B retry) -> 5000ms (7B one-shot success)
- **Better overall experience** (accuracy > speed)

---

#### 2.2 Ollama Performance Tuning

**Check current configuration**:
```bash
# Confirmed GPU enabled: --n-gpu-layers 37
# But other parameters may not be optimal

# Current
--ctx-size 1024        # May be too small
--batch-size 512       # Can be increased
--threads 4            # Jetson has 8 cores (4 online)
--parallel 2           # Concurrent request count
```

**Optimized configuration**:
```bash
# Edit /etc/systemd/system/ollama.service
[Service]
Environment="OLLAMA_NUM_PARALLEL=4"           # Increase concurrency
Environment="OLLAMA_MAX_LOADED_MODELS=2"      # Load 3B+7B simultaneously
Environment="OLLAMA_FLASH_ATTENTION=1"        # Enable Flash Attention
Environment="OLLAMA_NUM_GPU=99"               # Force all layers to GPU
Environment="OLLAMA_LLM_LIBRARY=cuda"         # Ensure CUDA backend

# Restart
sudo systemctl daemon-reload
sudo systemctl restart ollama
```

**Verify GPU usage**:
```bash
# Monitor during testing
watch -n 0.5 tegrastats

# Send LLM request
echo "立ってそして挨拶して" | ollama run claudia-go2-7b:v7

# Should see GR3D_FREQ 90%+
```

**Expected improvement**: Latency reduced by 30-50% (5000ms -> 2500-3500ms)

---

#### 2.3 Preload Models into Memory (Eliminate Cold Start)

**Problem**: First call needs to load model (+1-2s)

**Operation**:
```python
# Add warmup in ProductionBrain.__init__
async def _warmup_models(self):
    """Warm up models (load into GPU memory)"""
    warmup_commands = ["hello", "座って", "立ってそして挨拶"]

    for cmd in warmup_commands:
        # 7B warmup
        await self._call_ollama_v2(self.model_7b_pool[0], cmd, timeout=10)
        # 3B warmup (if still used)
        await self._call_ollama_v2(self.model_3b_pool[0], cmd, timeout=5)

    self.logger.info("Model warmup complete")

# Call during initialization
await self._warmup_models()
```

**Expected improvement**: Consistent first response time, no cold start penalty

---

### P2 - Architecture Upgrade (Complete Next Week)

#### 3.1 Hybrid Architecture: Local LLM + Cloud API Fallback

**Motivation**:
- Local LLM (Qwen 7B): Fast but limited understanding
- Cloud API (Claude/GPT-4): Slower but intelligent
- **Hybrid**: 90% local processing, 10% complex cases go to cloud

**Architecture**:
```python
class HybridBrain:
    def __init__(self):
        self.local_llm = ProductionBrain()  # Qwen 7B
        self.cloud_api = ClaudeAPI()        # Anthropic Claude

    async def process_command(self, cmd):
        # 1. Hot path (<1ms)
        if hotpath_hit := self._try_hotpath(cmd):
            return hotpath_hit

        # 2. Local LLM attempt (<3s)
        local_result = await self.local_llm.process(cmd, timeout=3)

        # 3. Confidence check
        if local_result.confidence > 0.8:
            return local_result  # Local success

        # 4. Cloud fallback (complex cases)
        self.logger.warning(f"Local LLM confidence low ({local_result.confidence}), using cloud API")
        cloud_result = await self.cloud_api.process(cmd, timeout=10)

        # 5. Cache cloud result (available locally next time)
        self._cache_cloud_result(cmd, cloud_result)

        return cloud_result
```

**Cost control**:
- Claude API: ~$0.003/request (1K tokens)
- 100 commands/day -> $0.30/day -> $9/month
- **Acceptable** (compared to robot hardware cost)

**Expected improvement**:
- Accuracy: 90% -> 99%
- Complex understanding capability: Excellent (Claude Sonnet level)
- Average latency: 2500ms (90% local) + 5000ms*10% (cloud) = 2750ms

---

#### 3.2 Fine-tuning a Dedicated Model (Reinforcement Learning)

**Current problem**: General-purpose Qwen model is not ideal for robot control

**Solution**: Fine-tune using Go2 actual interaction data
```bash
# Collect real dialog data
logs/audit/*.jsonl  # Existing audit logs

# Extract training samples
{
    "input": "立ってそして挨拶して",
    "output": {"r":"立ってから挨拶します","a":null,"seq":[1004,1016]},
    "feedback": "success"  # Whether user was satisfied
}

# Fine-tune Qwen 3B
python3 scripts/llm/finetune_qwen.py \
    --base-model Qwen/Qwen2.5-3B-Instruct \
    --data logs/audit/training_data.jsonl \
    --output models/claudia-go2-3b-v12 \
    --epochs 3

# Deploy
ollama create claudia-go2-3b:v12 -f models/claudia-go2-3b-v12
```

**Data requirements**: At least 1000 labeled samples (how many do we have currently?)

**Expected improvement**:
- 3B model accuracy: 60% -> 85%
- Japanese purity: Mixed -> 99% pure Japanese
- API mapping errors: Reduced by 90%

---

#### 3.3 Intelligent Dialog vs Action Intent Classification

**Problem**: Is "可愛いね" praise or a request for a cute action?

**Solution**: Add an intent classification layer
```python
def classify_intent(self, command: str) -> str:
    """
    Classify user intent

    Returns:
        - "pure_dialog": Pure dialog (e.g., "who are you")
        - "action_request": Action command (e.g., "座って")
        - "dialog_with_action": Dialog + action (e.g., "可愛いね" -> praise + cute action)
    """
    # Use lightweight classification model (BERT-tiny, <100ms)
    intent = self.intent_classifier.predict(command)

    if intent == "dialog_with_action":
        # Return both reply and action
        return BrainOutput(
            response="ありがとうございます！",
            api_code=1036,  # Heart
            reasoning="dialog_with_action"
        )
```

**Expected improvement**: Improved user satisfaction (more natural interaction)

---

## Implementation Priority

### Execute Immediately Today (P0)
1. Create v11.3 pure Japanese Modelfile (15 minutes)
2. Expand hot path to 50+ keywords (30 minutes)
3. Add common sequence predefinitions (15 minutes)
4. Test verification (30 minutes)

**Expected**: Eliminate language confusion, 80% hot path hit rate, partial performance improvement

---

### Complete This Week (P1)
1. Upgrade main model to 7B (1 hour testing + adjustment)
2. Ollama performance tuning (2 hours)
3. Model warmup mechanism (30 minutes)
4. A/B test verification (1 hour)

**Expected**: 90% accuracy, latency <3s, noticeably improved intelligence

---

### Next Week Planning (P2)
1. Design hybrid architecture (2 hours)
2. Integrate Claude API (3 hours)
3. Collect training data (ongoing)
4. Fine-tuning experiments (1 day)

**Expected**: Near commercial-grade intelligence

---

## Performance Metrics Comparison

| Metric | Current (v11.2) | After P0 | After P1 | P2 Target |
|------|-------------|----------|----------|--------|
| **Avg latency** | 3000ms | 500ms* | 2500ms | 2000ms |
| **Accuracy** | 60% | 65% | 90% | 99% |
| **Hot path hit rate** | 20% | 80% | 80% | 85% |
| **Language purity** | Chinese-Japanese mixed | Pure Japanese | Pure Japanese | Pure Japanese |
| **Sequence understanding** | Failed | Predefined | 7B success | Claude-level |
| **Cost/day** | $0 | $0 | $0 | $0.30 |

*P0 latency improvement mainly from hot path expansion (80% hit), non-hot-path still 3s

---

## Risks and Limitations

### Technical Limitations
1. **Jetson compute ceiling**: Orin NX cannot run 13B+ models
2. **Local LLM ceiling**: Qwen 7B understanding capability < Claude
3. **Network dependency**: Cloud fallback requires stable internet

### Resource Requirements
1. **Development time**: P0 (2 hours), P1 (1 day), P2 (3 days)
2. **Test data**: Need to collect 1000+ real dialog samples
3. **Cloud cost**: P2 solution ~$10/month API fees

### Recommendations
- **Execute P0 immediately**: Quickly improve user experience
- **Complete P1 this week**: Reach acceptable intelligence level
- **P2 as needed**: If P1 meets requirements, P2 can be deferred

---

**Author**: Claude Code
**Last Updated**: 2025-11-14 18:00 UTC
