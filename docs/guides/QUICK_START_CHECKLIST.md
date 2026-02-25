# Claudia LLM Brain Upgrade - Quick Start Checklist

## Implementation Status

### Completed (Track A + Track B)

- [x] **Async call fix**: `production_brain.py` uses `asyncio.to_thread`
- [x] **State monitoring integration**: Real-time IMU/battery/posture reading
- [x] **Safety validator**: `safety_validator.py` code-level enforcement rules
- [x] **Enhanced startup script**: `start_production_brain_v2.sh` comprehensive environment checks
- [x] **New Modelfile**: `ClaudiaIntelligent_Qwen7B` vocabulary-free design
- [x] **Deployment script**: `deploy_track_b.sh` automated new model creation
- [x] **A/B test script**: `test_ab_quick.py` 20 core scenario comparisons

---

## 10-Minute Verification Flow

### Step 1: Environment Preparation (2 minutes)
```bash
cd ~/claudia

# Install ollama Python library
pip3 install ollama

# Verify Ollama service
curl http://localhost:11434/api/tags
# If fails, start: nohup ollama serve > /tmp/ollama.log 2>&1 &

# Check existing models
ollama list | grep claudia
# Should see: claudia-go2-3b:v11.2, claudia-go2-7b:v7
```

### Step 2: Track A Verification - Simulation Mode (3 minutes)
```bash
# Start enhanced script
./start_production_brain_v2.sh
# Select: 1

# Test basic commands
Input: 座って
Expected: Returns JSON, api_code=1009

Input: 緊急停止
Expected: Fast response (<100ms), direct Stop execution

Input: quit
# Exit test
```

**Verification Points**:
- [ ] Script correctly loads ROS2 and Ollama environment
- [ ] Simulation mode responds to commands normally
- [ ] Emergency stop uses bypass channel (no LLM call)
- [ ] No async blocking errors

### Step 3: Track B Deployment (3 minutes)
```bash
# Automated deployment of new model
./deploy_track_b.sh

# Expected output:
# qwen2.5:7b exists (or auto-pulled)
# Created claudia-intelligent-7b:v1 successfully
# Test: "座って" -> 1009
# Test: "立って" -> 1004

# Verify model was created
ollama list | grep claudia-intelligent
# Should see: claudia-intelligent-7b:v1
```

### Step 4: A/B Comparison Test (2 minutes)
```bash
# Run quick comparison
python3 test_ab_quick.py

# Check key output metrics:
# [Baseline] claudia-go2-3b:v11.2
#   Accuracy: X%
#   Latency: Yms
#
# [New Model] claudia-intelligent-7b:v1
#   Accuracy: X%
#   Latency: Yms
#
# Comparison Results:
#   Accuracy change: +/- Z%
#   Latency change: +/- Wms
```

**Decision Criteria**:
- Pass: Accuracy >= baseline -5%, Latency <= 2000ms
- Needs optimization: Accuracy drop >5% or Latency >2000ms
- Fail: JSON compliance rate <100% or many errors

---

## Key Test Cases

### Basic Commands (Must be 100%)
```
座って → 1009 (Sit)
立って → 1004 (Stand)
止まって → 1003 (Stop)
```

### Semantic Understanding (Track B Core Advantage)
```
疲れた → 1009 (Sit)     # "Tired" should be understood as rest -> sit down
可愛い → 1036 (Heart)    # "Cute" should be understood as heart gesture
元気 → 1023 (Dance)      # "Energetic" should be understood as dance
挨拶して → 1016 (Hello)  # "Greet" should be understood as Hello action
```

### Exception Handling
```
Empty input → api_code=None or 0
Nonsense input → api_code=None or 0
```

---

## Expected Results Comparison

### Track A (Conservative Improvement)
| Metric | Before Improvement | After Improvement | Notes |
|--------|-------------------|-------------------|-------|
| Accuracy | 85% | 85% | Model unchanged, accuracy unchanged |
| First-call latency | ~3000ms | ~500ms | LLM warmup |
| Emergency stop latency | ~800ms | <100ms | Bypass channel |
| Low battery protection | No | Yes | Code-level validation |
| Posture auto-adaptation | No | Yes | Auto-complete prerequisite actions |

### Track B (Aggressive Refactoring)
| Metric | v11.2 Baseline | v1 Target | Expectation |
|--------|---------------|-----------|-------------|
| Basic commands | 100% | >=95% | Slight tolerance |
| Semantic understanding | ~40% | >=70% | **Core improvement** |
| Multilingual | 66% | >=80% | Support for English/Chinese |
| Average latency | 800ms | <=2000ms | 7B model slightly slower |

---

## Common Issues Quick Reference

### Q1: pip3 install ollama fails
```bash
# Use domestic mirror
pip3 install ollama -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### Q2: Ollama service not responding
```bash
# Check process
ps aux | grep ollama
# Restart service
killall ollama
nohup ollama serve > /tmp/ollama.log 2>&1 &
sleep 5
curl http://localhost:11434/api/tags
```

### Q3: ROS2 Topics not visible
```bash
# Confirm environment variables
echo $RMW_IMPLEMENTATION  # Should be rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash
source .env.ros2
ros2 topic list
```

### Q4: Model loading timeout
```bash
# First load of 7B model is slow (~5 seconds), this is normal
# Optional: Preload
ollama run claudia-intelligent-7b:v1 "test" >/dev/null 2>&1 &
```

### Q5: A/B test errors
```bash
# Confirm both models exist
ollama list | grep -E "(claudia-go2-3b:v11.2|claudia-intelligent-7b:v1)"

# If v11.2 is missing
ollama pull claudia-go2-3b:v11.2

# If v1 is missing, redeploy
./deploy_track_b.sh
```

---

## File Index

### Core Code
- [src/claudia/brain/production_brain.py](src/claudia/brain/production_brain.py) - Main brain logic
- [src/claudia/brain/safety_validator.py](src/claudia/brain/safety_validator.py) - Safety validator

### Startup Scripts
- [start_production_brain_v2.sh](start_production_brain_v2.sh) - Enhanced startup script (recommended)
- [start_production_brain.sh](start_production_brain.sh) - Legacy startup script

### Track B Files
- [ClaudiaIntelligent_Qwen7B](ClaudiaIntelligent_Qwen7B) - New Modelfile definition
- [deploy_track_b.sh](deploy_track_b.sh) - Automated deployment script
- [test_ab_quick.py](test_ab_quick.py) - A/B quick test

### Documentation
- [LLM_BRAIN_UPGRADE_GUIDE.md](LLM_BRAIN_UPGRADE_GUIDE.md) - Complete technical guide
- [QUICK_START_CHECKLIST.md](QUICK_START_CHECKLIST.md) - This document
- [README_FINAL_SOLUTION.md](README_FINAL_SOLUTION.md) - SDK mutual exclusion solution

---

## Next Steps

### Execute Immediately (Today)
1. Execute Step 1-4 complete verification (10 minutes)
2. Review A/B test results, decide whether to launch Track B
3. If Track B passes: Consider 30% gradual rollout test

### Short-term (1-2 days)
- If semantic understanding improvement is significant (>20%) -> Expand gradual rollout to 70%
- If latency is too high (>2000ms) -> Consider optimization or rollback to 3B
- If semantic understanding improvement is limited (<10%) -> Consider few-shot expansion or LoRA fine-tuning

### Medium-term (1 week)
- Real hardware verification (caution: ensure surrounding safety)
- Collect actual usage data
- Continuously monitor error rate and performance

---

## Verification Pass Criteria

```
[Pass] ollama Python library installed
[Pass] Ollama service running
[Pass] Track A simulation mode test passed
[Pass] Track B model created successfully
[Pass] A/B test completed, accuracy >= target
[Pass] Latency meets expectations (<=2000ms)
[Pass] No blocking/crash errors

-> Ready to enter real hardware test phase
```

---

**Created**: 2025-11-13
**Estimated Execution Time**: 10 minutes
**Prerequisites**: Jetson Orin NX + ROS2 Foxy + Ollama 0.9.5+
