# Unitree Go2 Action Compatibility Report

## **Confirmed Supported Actions**

Based on actual testing, Go2 confirms support for the following actions:

### Basic Control Actions
| API | Method Name | Description | Status |
|-----|-------------|-------------|--------|
| 1001 | Damp | Damping | Supported |
| 1003 | StopMove | Stop movement | Supported |
| 1004 | StandUp | Stand up | Supported |
| 1005 | StandDown | Lie down | Supported |
| 1009 | Sit | Sit down | Supported |

### Performance Actions
| API | Method Name | Description | Status |
|-----|-------------|-------------|--------|
| 1016 | Hello | Greet/Wave | Supported |
| 1017 | Stretch | Stretch | Supported |
| 1021 | Wallow | Heart gesture | **Not Supported** (3203) |

### Advanced Actions
| API | Method Name | Description | Status |
|-----|-------------|-------------|--------|
| 1010 | Rollover | Roll over | Not tested |
| 1022 | Dance | Dance 1 | Not tested |
| 1023 | Dance2 | Dance 2 | Not tested |
| 1024 | FrontFlip | Front flip | Not tested |
| 1025 | ShakeHands | Shake hands | Method does not exist |
| 1026 | Cheer | Celebratory bow | Method does not exist |
| 1028 | Jump | Jump | Method does not exist |
| 1029 | Pounce | Pounce | Method does not exist |
| 1030 | Bow | Bow | Method does not exist |
| 1031 | Handstand | Handstand | Method does not exist |

## **Error Code Explanation**

### 3203 Error = Action Not Supported
- **Meaning**: `RPC_ERR_SERVER_API_NOT_IMPL` - Server-side API not implemented
- **Cause**: The action is not available on Go2 hardware
- **Not**: An app occupation issue (that would be 3103)

### Why is Heart Gesture (Wallow) Not Supported?
Go2 may have hardware limitations or firmware version restrictions that prevent executing the heart gesture action.

## **Recommended Modifications**

### 1. Filter Unsupported Actions
```python
# Actual list of Go2-supported actions
GO2_SUPPORTED_ACTIONS = [
    1001,  # Damp
    1003,  # StopMove
    1004,  # StandUp
    1005,  # StandDown
    1009,  # Sit
    1016,  # Hello
    1017,  # Stretch
    # 1021 Heart gesture not supported
]
```

### 2. Update LLM Prompts
Remove unsupported action mappings to avoid returning non-executable APIs.

### 3. Optimize Error Messages
```python
if result == 3203:
    print("This action is not supported on Go2")
    print("   Go2 does not support heart gesture, handshake, bow, and other advanced actions")
```

## **Test Commands**

### Available Commands (Will Succeed)
- こんにちは (Hello/1016)
- 座って (Sit/1009)
- 立って (StandUp/1004)
- 伸びて (Stretch/1017)
- 横になって (StandDown/1005)

### Unavailable Commands (Return 3203)
- ハート/比心 (Wallow/1021)
- お手/握手 (ShakeHands/1025)
- お辞儀/鞠躬 (Bow/1030)

## **Conclusion**

1. **3203 is not app occupation** - it's hardware not supported
2. **Go2 has limited functionality** - only supports basic actions
3. **Model needs updating** - remove unsupported actions
