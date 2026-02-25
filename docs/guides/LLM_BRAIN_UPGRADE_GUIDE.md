# Claudia LLM Brain Upgrade Guide - Track A+B Implementation

## Project Background

**Current Problem Diagnosis**:
- Existing LLM system is essentially a "keyword extractor", not true semantic understanding
- Modelfile contains 175 lines of hardcoded keyword mappings (座って->1009, 立って->1004...)
- Lacks state awareness and safety validation layers
- Cannot understand context and semantically similar words (e.g., "疲れた" -> rest -> Sit)

**Upgrade Goal**:
Achieve a truly "vocabulary-free" intelligent brain, reaching the offline semantic understanding level of Unitree App's "Silly Dog"

---

## Dual-Track Parallel Approach

### Track A: Conservative Improvement (Compatible with Existing Models)
**Goal**: Improve system safety and stability without changing the model

#### Implemented Improvements:

1. **Async Call Optimization** ([production_brain.py:454-503](src/claudia/brain/production_brain.py#L454-L503))
   - Wraps synchronous LLM calls with `asyncio.to_thread`
   - Prevents event loop blocking
   - Added timeout protection (default 10 seconds)

2. **State Awareness Integration** ([production_brain.py:229-254](src/claudia/brain/production_brain.py#L229-L254))
   - Integrated `SystemStateMonitor` for real-time IMU/battery/posture reading
   - Checks robot current state before each command execution
   - Battery unit normalization (automatically handles both 0-1 and percentage formats)

3. **Code-Level Safety Validation** ([safety_validator.py](src/claudia/brain/safety_validator.py))
   - **Rule 1**: Emergency state only allows Stop command
   - **Rule 2**: Battery <5% only allows Stop and Sit
   - **Rule 3**: Battery <20% prohibits high-energy actions (Flip/Jump/Pounce)
   - **Rule 4**: Auto-completes prerequisite actions (auto-inserts StandUp when standing is needed)
   - **Rule 5**: Rejects execution in error state

4. **Enhanced Startup Script** ([start_production_brain_v2.sh](start_production_brain_v2.sh))
   - Explicit ROS2 Foxy environment sourcing
   - CycloneDDS configuration validation (eth0, Domain 0)
   - Automatic Ollama service check and startup
   - Model availability pre-check
   - LLM warmup (first call latency optimization)
   - Network connectivity test (ping 192.168.123.161)

5. **Emergency Bypass System** ([production_brain.py:520-532](src/claudia/brain/production_brain.py#L520-L532))
   - Commands like "緊急停止"/"stop" bypass LLM and execute directly
   - Reduces stop command latency (<50ms)

#### Key Fixes:

| Issue | Fix Approach | Code Location |
|------|---------|---------|
| Event loop blocking | `asyncio.to_thread(ollama.chat)` | production_brain.py:467 |
| Undefined timer variable | `start_time` moved to function entry | production_brain.py:506 |
| Inconsistent battery units | Normalization check `>1.0 then /100` | safety_validator.py:90-92 |
| Sequence/single-step ambiguity | `should_use_sequence_only` flag | safety_validator.py:126 |
| High-risk actions enabled by default | `enable_high_risk_actions=False` | production_brain.py:244 |

---

### Track B: Aggressive Refactoring (Vocabulary-Free)
**Goal**: Completely remove keyword mapping, achieve true semantic understanding using capability descriptions

#### New Modelfile Design ([ClaudiaIntelligent_Qwen7B](ClaudiaIntelligent_Qwen7B))

**Core Changes**:

```diff
- OLD (v11.2):
  sit->{"r":"座ります","a":1009}
  stand->{"r":"立ちます","a":1004}
  hello->{"r":"こんにちは","a":1016}
  ...(175 lines of keyword table)

+ NEW (v1):
  【利用可能な動作】
  - Sit(1009): 座る。疲れた時や休息したい時に使用
  - Stand(1004): 立ち上がる。動作開始前の基本姿勢
  - Hello(1016): 手を振って挨拶する
  ...(capability descriptions)

  【Few-shot例】
  Q: 疲れた
  A: {"intent":"休息したい","action":"Sit","api_code":1009}

  Q: 可愛い動作して
  A: {"intent":"可愛らしいジェスチャー","action":"Heart","api_code":1036}
```

**New Features**:
1. **Semantic reasoning**: Understands intent through action descriptions (疲れた -> rest -> Sit)
2. **Context understanding**: Can explain why a particular action was chosen ("intent" field)
3. **Embedded physical constraints**: Battery/posture limits declared in Modelfile
4. **Few-shot learning**: Provides typical semantic understanding cases

#### Deployment Tool ([deploy_track_b.sh](deploy_track_b.sh))

```bash
# Automated deployment flow
./deploy_track_b.sh
# 1. Pull qwen2.5:7b base model
# 2. Create claudia-intelligent-7b:v1
# 3. Run sanity tests
```

#### A/B Test Script ([test_ab_quick.py](test_ab_quick.py))

**Test Matrix** (20 core scenarios):
- **Basic exact match** (5): 座って, 立って, ハート, ダンス, stop
- **Semantic understanding** (10): 疲れた->Sit, 可愛い->Heart, 元気->Dance...
- **Multilingual** (3): sit, cute, etc.
- **Exception handling** (2): Empty input, nonsense input

**Acceptance Criteria**:
- Accuracy >= baseline model -5% (slight decrease allowed)
- Average latency <= 2000ms
- JSON compliance rate 100%

---

## Implementation Steps

### Phase 1: Environment Preparation

```bash
cd ~/claudia

# 1. Install Python ollama library
pip3 install ollama

# 2. Verify Ollama service
curl http://localhost:11434/api/tags
ollama list | grep claudia

# 3. Verify ROS2 environment
source /opt/ros/foxy/setup.bash
ros2 topic list | grep sportmodestate

# 4. Network connectivity
ping -c 1 192.168.123.161
```

### Phase 2: Track A Verification (Simulation Mode)

```bash
# 1. Use new startup script
./start_production_brain_v2.sh
# Select: 1

# 2. Test core functions
Input: 座って
Expected: JSON returned, api_code=1009

Input: 緊急停止
Expected: Fast response (<100ms), direct Stop execution

Input: quit
# Exit test
```

### Phase 3: Track B Deployment and Testing

```bash
# 1. Deploy new model
./deploy_track_b.sh

# 2. Run A/B comparison
python3 test_ab_quick.py

# 3. Check key semantic understanding cases
Expected improvements:
  "疲れた" -> 1009 (Sit)
  "可愛い" -> 1036 (Heart)
  "元気" -> 1023 (Dance)
```

### Phase 4: Real Hardware Verification (Cautious)

```bash
# Only execute after simulation mode tests pass

# 1. Hardware preparation
- Robot battery >50%
- Clear 2m radius activity space
- Close Unitree App
- Prepare remote controller emergency stop

# 2. Start hardware mode
./start_production_brain_v2.sh
# Select: 2) Real hardware mode

# 3. Basic action verification
座って -> Observe robot sitting
立って -> Observe robot standing
止まって -> Immediate stop

# 4. Semantic understanding verification
疲れた -> Should sit (not error)
挨拶して -> Should Hello (not no response)
```

---

## Acceptance Criteria

### Track A (Must pass 100%)

| Item | Criteria | Verification Method |
|------|------|---------|
| Async calls | No blocking, timeout works | Long command doesn't freeze interface |
| State reading | Correctly reads IMU/battery | Print current state log |
| Safety rules | Low battery rejects high-energy actions | Simulate 5% battery test |
| Emergency bypass | "stop" response <100ms | Timer verification |
| Posture auto-complete | Hello from sitting auto-stands | Observe action sequence |

### Track B (Targets)

| Metric | Baseline (v11.2) | New Model (v1) | Status |
|------|------------|-----------|------|
| Basic command accuracy | 100% (5/5) | >=95% | Pending test |
| Semantic understanding accuracy | ~40% (4/10) | >=70% | Pending test |
| Multilingual accuracy | 66% (2/3) | >=80% | Pending test |
| Average latency | ~800ms | <=2000ms | Pending test |
| JSON compliance | 100% | 100% | Pending test |

---

## Gradual Rollout Strategy

### Stage 1: Gradual Testing (0% -> 30%)
```python
# Add traffic splitting in production_brain.py
import random
if random.random() < 0.3:
    result = await self._call_ollama_v2("claudia-intelligent-7b:v1", command)
else:
    result = await self._call_ollama_v2("claudia-go2-3b:v11.2", command)
```

**Monitoring Metrics**:
- Accuracy changes
- Latency P95/P99
- Error rate

**Decision Points**:
- If accuracy improves >10% -> Expand to 70%
- If latency P95 >3000ms -> Rollback

### Stage 2: Expand Gradual Rollout (30% -> 70%)
- Continue monitoring for 1 week
- Collect user feedback (if available)
- Analyze failure cases

### Stage 3: Full Rollout (70% -> 100%)
- Review all metrics before final decision
- Keep v11.2 as a degradation option

---

## Known Issues and TODOs

### Track A
- [x] Event loop blocking
- [x] Battery unit normalization
- [x] Sequence execution ambiguity
- [ ] Test cases need actual state injection (currently using text markers)
- [ ] Dynamic toggle for high-risk actions (currently hardcoded False)

### Track B
- [ ] Few-shot cases may need expansion (currently only 2)
- [ ] 7B model first load latency is high (~3-5 seconds)
- [ ] May need LoRA fine-tuning if semantic understanding is unsatisfactory
- [ ] Multilingual support pending verification (English/Chinese)

---

## Key File List

### New Files
```
~/claudia/
+-- start_production_brain_v2.sh      # Track A enhanced startup script
+-- ClaudiaIntelligent_Qwen7B         # Track B new Modelfile
+-- deploy_track_b.sh                 # Track B auto deployment
+-- test_ab_quick.py                  # A/B quick comparison test
+-- LLM_BRAIN_UPGRADE_GUIDE.md        # This document
```

### Modified Files
```
src/claudia/brain/
+-- production_brain.py               # Async calls + state integration + safety validation
+-- safety_validator.py               # New: code-level safety rules
```

### Configuration Files
```
.env.ros2                              # ROS2 environment variables (need source)
cyclonedds.xml                         # DDS configuration (eth0)
```

---

## Technical Details

### Ollama Python Library Usage
```python
import ollama

# Check model existence
ollama.show("claudia-go2-3b:v11.2")

# Structured output
response = ollama.chat(
    model="claudia-go2-3b:v11.2",
    messages=[{'role': 'user', 'content': '座って'}],
    format='json',  # Force JSON output
    options={
        'temperature': 0.1,   # Low temperature for stability
        'num_predict': 30,    # Limit token count
        'top_p': 0.7
    }
)
data = json.loads(response['message']['content'])
```

### State Monitor Integration
```python
from claudia.robot_controller.system_state_monitor import (
    create_system_state_monitor, SystemStateInfo, SystemState
)

monitor = create_system_state_monitor(
    node_name="claudia_brain_monitor",
    update_rate=5.0  # 5Hz update rate
)
monitor.initialize()
monitor.start_monitoring()

# Read current state
state = monitor.get_current_state()
print(f"Battery: {state.battery_level*100:.1f}%")
print(f"Standing: {state.is_standing}")
print(f"System: {state.state.name}")
```

### Safety Validator Usage
```python
from claudia.brain.safety_validator import get_safety_validator

validator = get_safety_validator(enable_high_risk=False)
result = validator.validate_action(api_code=1030, state=current_state)

if not result.is_safe:
    print(f"Rejected: {result.reason}")
elif result.modified_sequence:
    print(f"Auto-completed: {result.modified_sequence}")
    if result.should_use_sequence_only:
        api_code = None  # Only execute sequence, avoid duplication
```

---

## Troubleshooting

### Ollama Service Not Connecting
```bash
curl http://localhost:11434/api/tags
# If fails:
ps aux | grep ollama
killall ollama
nohup ollama serve > /tmp/ollama.log 2>&1 &
```

### Model Does Not Exist
```bash
ollama list | grep claudia
# If missing:
ollama pull claudia-go2-3b:v11.2
ollama pull qwen2.5:7b
```

### ROS2 Topics Not Visible
```bash
echo $RMW_IMPLEMENTATION  # Should be rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash
source .env.ros2
ros2 topic list
```

### Safety Validation False Positive
```bash
# Temporarily disable high-risk check
# Modify production_brain.py:244
self.safety_validator = get_safety_validator(enable_high_risk=True)
```

---

## Next Actions

### Execute Immediately
1. Code implementation complete (Track A+B)
2. Install ollama library: `pip3 install ollama`
3. Track A simulation test
4. Track B deployment and A/B test

### Short-term (1-2 days)
- Collect A/B test data
- Decide whether to launch Track B
- Fix discovered issues

### Medium-term (1 week)
- Real hardware verification
- Performance tuning (if latency too high)
- Consider LoRA fine-tuning (if semantic understanding insufficient)

---

## Reference Documents

- [Ollama Python Library](https://github.com/ollama/ollama-python)
- [Qwen2.5 Model Card](https://huggingface.co/Qwen/Qwen2.5-7B-Instruct)
- `docs/CORRECT_LLM_ARCHITECTURE.md` - Original architecture design
- `docs/SDK_LIMITATION_ANALYSIS.md` - SDK limitation analysis
- `test/README.md` - Test framework documentation

---

**Last Updated**: 2025-11-13
**Implementer**: Claude Code
**Status**: Code complete, pending test verification
