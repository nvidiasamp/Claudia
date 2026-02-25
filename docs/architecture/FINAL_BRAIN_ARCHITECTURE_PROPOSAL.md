# Claudia Brain Architecture: Final Proposal

## Core Design Decisions

### 1. **Direct API Output**
**Decision**: LLM directly outputs API codes, avoiding secondary mapping
```json
// Input: "比心" (heart gesture)
// Output: {"response": "ハートします", "api_code": 1021}
// Direct execution: SportClient.Wallow()
```

### 2. **3B Primary, 7B Secondary**
**Decision**: 95% handled by 3B for fast response, 5% by 7B for deep understanding
- 3B: 1-2 seconds, handles routine commands
- 7B: 3-5 seconds, handles complex sequences

### 3. **Minimal Output Format**
**Decision**: Minimized JSON structure
```json
{
  "response": "Japanese TTS reply",
  "api_code": single API code,
  "sequence": [API sequence]  // optional
}
```

### 4. **Three-Layer Optimization Strategy**
1. **Cache layer**: 0 latency, direct return for common commands
2. **Mapping layer**: <50ms, direct keyword mapping
3. **LLM layer**: 1-2 seconds, intelligent understanding

## Recommended Architecture Implementation

```python
# Priority processing flow
async def process(command):
    # 1. Cache check (0ms)
    if command in cache:
        return cache[command]

    # 2. Direct mapping (<50ms)
    if has_keyword(command):
        return direct_map(command)

    # 3. 3B model (1-2s)
    if is_simple(command):
        return call_3b(command)

    # 4. 7B model (3-5s)
    if is_complex(command):
        return call_7b(command)
```

## Performance Expectations

| Command Type | Processing Method | Response Time | Success Rate |
|---------|---------|---------|--------|
| Common commands | Cache | 0ms | 100% |
| Simple commands | 3B model | 1-2s | 95% |
| Complex sequences | 7B model | 3-5s | 85% |
| Ambiguous commands | 3B + mapping | 1-2s | 80% |

## Immediate Action Plan

### Phase 1: 3B Model Optimization (Immediate)
1. Create minimal prompt
2. Direct API mapping table
3. Test common commands

### Phase 2: Hybrid Architecture Implementation (Today)
1. Implement HybridBrain class
2. Integrate caching mechanism
3. Test complex sequences

### Phase 3: Real Hardware Integration (Tomorrow)
1. Connect SportClient
2. Test all 26 actions
3. Optimize response time

## Key Innovations

### 1. **State-Aware Sequences**
```json
// "Sit down then greet"
{
  "response": "座って挨拶します",
  "sequence": [1009, 1004, 1016]  // Sit -> Stand -> Wave
}
```

### 2. **Intelligent Degradation Mechanism**
- 7B timeout -> Degrade to 3B
- 3B failure -> Degrade to mapping
- Mapping failure -> Return error

### 3. **Progressive Optimization**
- Collect frequently used user commands
- Dynamically update cache
- Continuously optimize prompts

## Answers to Key Questions

### Q: Is the reasoning field necessary?
**A: Not necessary**. Production should remove it and only keep essential fields.

### Q: How to map to API?
**A: Directly output API codes**, no secondary mapping needed.

### Q: 3B or 7B?
**A: 3B primary**, 7B only for complex scenarios.

### Q: How to handle complex sequences?
**A: Use sequence arrays**, supporting multi-step and delayed execution.

## Conclusion

This architecture perfectly balances:
- **Response speed** (3B fast path)
- **Intelligence level** (7B backup)
- **Practicality** (Direct API)
- **Extensibility** (Sequence support)

**This is what a true AI robot brain looks like!**
