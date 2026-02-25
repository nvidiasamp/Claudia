# SDK Functionality Limitation Analysis - Why the App Can Do It But SDK Cannot

## **Core Issue**

### **It's Not a Hardware Limitation, It's an SDK Limitation!**

According to the official GitHub Issue #63 reply:
> "The Python SDK does not currently support this. The official documentation corresponds to the C++ SDK. It is recommended to use the DDS-based C++ SDK for development."
> -- blogdefotsec (Unitree Official)

## **Three-Layer Architecture Comparison**

```
+----------------+
| Remote/App     | --> Uses internal C++ API (full functionality)
+----------------+
        |
+----------------+
| Robot Hardware  | --> Supports all actions
+----------------+
        |
+----------------+
| Python SDK     | --> Only wraps partial APIs (incomplete functionality)
+----------------+
```

## **Features Missing from Python SDK**

### Officially Confirmed Missing APIs (Issue #63):
- `SwitchMoveMode` - Mode switching
- `HandStand` - Handstand
- `MoveToPos` - Move to position
- Possibly also: `Wallow` (heart gesture), `ShakeHands` (handshake), etc.

### Features PR #76 Attempted to Add:
```python
# New V2.0 APIs (ID 2044-2058)
- HandStand (handstand)
- ClassicWalk (classic walking)
- FreeBound (free bounding)
- FreeJump (free jumping)
- FreeAvoid (free obstacle avoidance)
- WalkUpright (upright walking)
- CrossStep (cross stepping)
```

**However, this PR may not have been merged into the main branch yet!**

## **Solutions**

### Solution 1: Use C++ SDK (Recommended)
```cpp
// C++ SDK has full functionality
#include "unitree_sdk2/sport/sport_client.hpp"
client.Wallow();  // Heart gesture available
client.HandStand(); // Handstand available
```

### Solution 2: Wait for Python SDK Update
- Monitor the merge status of PR #76
- Or manually apply the PR's changes

### Solution 3: Extend Python SDK Yourself
```python
# Manually add missing APIs
class SportClient:
    def Wallow(self):
        # Send API ID 1021
        return self._call(1021)
```

### Solution 4: Use DDS Communication Directly
```python
# Bypass SDK, send DDS messages directly
# Requires understanding of the underlying protocol
```

## **New Interpretation of 3203 Error**

```python
3203 = RPC_ERR_SERVER_API_NOT_IMPL
```

- **Previous understanding**: Robot does not support the action (incorrect)
- **Correct understanding**: Python SDK has not implemented the API wrapper

The robot hardware **does actually support** these actions (the fact that the remote controller can execute them proves this) -- the Python SDK just hasn't provided the interface!

## **Verification Methods**

### Check SDK Version
```python
# Check if the current SDK contains these methods
import unitree_sdk2py.go2.sport.sport_client as sc
print(dir(sc.SportClient))  # List all methods
```

### Try Manual Calls
```python
# Even if the method doesn't exist, you can try calling the API directly
sport_client._call(1021)  # Heart gesture API
sport_client._call(1031)  # Handstand API
```

## **Key Findings Summary**

1. **Remote/App** = Uses the complete internal C++ API
2. **Python SDK** = Only a partial wrapper, functionality is incomplete
3. **3203 Error** = SDK limitation, not hardware limitation
4. **Go2 Hardware** = Actually supports all actions available on the remote controller

## **Recommended Actions**

### Short-term Solution
1. Only use methods available in the Python SDK
2. Filter out unsupported actions

### Long-term Solution
1. Migrate to C++ SDK
2. Or wait for Python SDK updates
3. Or extend the missing APIs yourself

---

**Conclusion: It's not that Go2 doesn't support heart gesture -- the Python SDK hasn't wrapped this functionality!**
