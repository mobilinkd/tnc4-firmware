# Issue #2: TX Confirmation — Investigation & Implementation Plan

> **For Hermes:** Execute this investigation, then implement based on findings.

**Goal:** Add delivery confirmation from the modulator back to TX callers so the digipeater can track frame lifecycle.

**Status:** Research phase. Design is NOT locked. This plan defines the investigation and implementation candidates.

---

## Architecture (Current)

```
Caller (IOEventTask)                     Modulator (hdlc::Encoder::process)
│                                        │
├─ acquire() IoFrame*                    │
├─ populate frame data                   │
├─ osMessagePut(hdlcOutputQueueHandle,   │
│     frame, osWaitForever)   ──────►    ├─ osMessageGet(input_, osWaitForever)
│                                        ├─ do_csma() → if timeout: release(), RETURN
│  (caller has NO feedback)              ├─ send preamble + frame data
│                                        ├─ release(frame)
│                                        └─ send tail
```

**The gap:** The modulator is fire-and-forget. Callers have zero visibility into:
- CSMA timeout → frame silently dropped
- TX success → frame transmitted, no notification  
- TX failure → DAC underrun, frame aborted, no notification

**Why this matters:**
- Ref-counted digipeater frames (issue #1) — digipeater needs to know when to call `release()`. Without confirmation, either leaks frames or releases too early.
- Statistics — operators want to know their TX success rate.
- KISS ACK Mode (issue #3) — blocked on TX confirmation. Can't ACK what you can't confirm.

---

## Phase 1: Investigation

### Task 1: Identify all TX call sites

**Goal:** Find every code path that enqueues an IoFrame to the modulator.

```bash
rg "hdlcOutputQueueHandle" --no-filename -n | grep -v "extern\|static\|osMessageQDef"
```

Expected: 2-5 call sites across IOEventTask, Digipeater (stub), and BERT/test paths.

### Task 2: Determine digipeater completion requirements

**Questions to resolve:**
1. Does the digipeater need synchronous or async confirmation?
2. Does it need per-frame success/fail or just aggregate stats?
3. Is retry needed? (Likely no — digipeated frames use p=0 CSMA, no retry.)
4. What's the frame lifecycle: acquire → populate → add_ref → enqueue → wait for confirm → release?

**Approach:** Check `Digipeater.cpp` stub, `tnc4-digipeater-integration-plan.md` (if available in workspace), and memory/hindsight for design docs.

### Task 3: Evaluate completion delivery mechanisms

| Mechanism | Memory | Latency | Complexity | Notes |
|-----------|--------|---------|------------|-------|
| **A. Status field on Frame** | 1 byte | N/A (poll) | Minimal | Caller must poll. Race-prone without synchronization. |
| **B. Callback in Frame** | 4 bytes | Immediate | Medium | Function pointer per frame. 48 frames × 4 = 192 bytes. Needs careful ISR safety. |
| **C. Return queue per caller** | ~80B/queue | Near-immediate | Medium | Natural FreeRTOS pattern. Caller provides queue handle. Modulator posts result. |
| **D. Task notification** | 0 | Immediate | Low | `osSignalSet` per task. Only one notification per task. Limits concurrent waiters. |
| **E. Completion queue on Frame** | 4 bytes + queue | Near-immediate | Medium | Frame stores optional queue handle (0 = no notification). Modulator posts TxResult. |

### Task 4: Memory budget analysis

Current IoFrame size: `data_` (SegmentedBuffer, ~variable) + `crc_` (4) + `fcs_` (4) + `complete_` (1) + `frame_type_` (1) = ~10 bytes overhead + list hook (8 bytes).

Adding to the 48-frame pool:
- 1 byte status field: 48 bytes
- 4 byte callback pointer: 192 bytes
- 4 byte queue handle: 192 bytes
- Independent return queue: ~80 bytes per caller

ST-BSS available: ~69KB. All options are well within budget.

---

## Phase 2: Design Decision

Based on investigation, select and document the chosen mechanism with rationale. Expected choice: **Option E (completion queue on Frame)** — zero overhead for callers that don't care, minimal footprint (192 bytes), natural FreeRTOS pattern.

### Design sketch (Option E):

```cpp
// In HdlcFrame.hpp — add to IoFrame:
enum class TxResult : uint8_t {
    NONE = 0,       // Not set / not yet transmitted
    SENT = 1,       // Frame transmitted successfully
    CSMA_TIMEOUT = 2, // CSMA timed out, frame dropped
    ABORTED = 3,    // Transmission aborted (DAC underrun)
};

struct TxConfirm {
    TxResult result;
    IoFrame* frame;
};

// Frame gains:
osMessageQId tx_completion_queue_{0};
TxResult tx_result_{TxResult::NONE};

// Modulator (hdlc::Encoder::process):
void process(IoFrame* frame) {
    // ...
    if (!do_csma()) {
        frame->tx_result(TxResult::CSMA_TIMEOUT);
        notify_completion(frame);
        release(frame);
        return;
    }
    // ... transmit ...
    frame->tx_result(TxResult::SENT);
    notify_completion(frame);
    release(frame);
}

void notify_completion(IoFrame* frame) {
    auto q = frame->tx_completion_queue();
    if (q) {
        TxConfirm confirm{frame->tx_result(), frame};
        osMessagePut(q, *(uint32_t*)&confirm, 0);
    }
}
```

---

## Phase 3: Implementation

### Task 5: Add TxResult and TxConfirm types

**Files:**
- Create: `Core/TNC/TxConfirm.hpp`

**Step 1:** Define `TxResult` enum and `TxConfirm` struct.

```cpp
#pragma once

#include "HdlcFrame.hpp"
#include <cstdint>

namespace mobilinkd { namespace tnc { namespace hdlc {

enum class TxResult : uint8_t {
    NONE = 0,
    SENT = 1,
    CSMA_TIMEOUT = 2,
    ABORTED = 3,
};

struct TxConfirm {
    TxResult result;
    IoFrame* frame;
};

}}} // mobilinkd::tnc::hdlc
```

**Step 2:** Commit.

### Task 6: Add completion queue + status to Frame

**Files:**
- Modify: `Core/TNC/HdlcFrame.hpp`

Add to `Frame` class:
- `osMessageQId tx_completion_queue_` (initialized to 0)
- `TxResult tx_result_` (initialized to NONE)
- Accessors: `tx_completion_queue()`, `set_tx_completion_queue(q)`, `tx_result()`, `tx_result(r)`

Update `clear()` to reset both.

**Verification:** Build succeeds.

### Task 7: Add completion notification to hdlc::Encoder::process()

**Files:**
- Modify: `Core/TNC/HDLCEncoder.hpp`

Add `notify_completion(IoFrame* frame)` private method.
In `process()`:
- After CSMA timeout: set `CSMA_TIMEOUT`, notify, release, return
- After successful TX: set `SENT`, notify, release
- (Abort path TBD)

**Verification:** Build succeeds. Walk through code paths to verify all release() sites.

### Task 8: Wire up digipeater completion

**Files:**
- Modify: `Core/TNC/Digipeater.cpp`

After digipeater rewrites and enqueues frame for TX:
- Set `frame->set_tx_completion_queue(digipeaterQueueHandle)`
- In digipeater task loop, handle `TxConfirm` messages

**Verification:** Build succeeds. Logic audit — digipeater correctly releases ref-counted frames on completion.

### Task 9: Update M17Encoder for parity

**Files:**
- Modify: `Core/TNC/M17Encoder.h`

Same completion pattern. M17Encoder also calls `release()` without notification. Add completion notification to maintain consistency.

**Verification:** Build succeeds.

### Task 10: Audit all release() call sites

**Goal:** Every call site that releases a frame after TX must have completion notification.

Expected call sites with release after TX:
1. `HDLCEncoder.hpp:207` — CSMA timeout → release ✓ (task 7)
2. `HDLCEncoder.hpp:220` — after TX → release ✓ (task 7)
3. `M17Encoder.h` — after TX → release ✓ (task 9)
4. `IOEventTask.cpp:612` — queue full → release (caller-side, no completion needed — frame never made it to modulator)

### Task 11: Full build verification

```bash
cd tnc4-firmware
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -S. -Bbuild/Debug -G Ninja
cmake --build build/Debug --target tnc4-firmware
```

Expected: zero errors, 0 warnings.

### Task 12: Update design docs

- Update `tnc4-digipeater-integration-plan.md` with TX confirmation mechanism
- Document TxResult types and expected digipeater behavior

---

## Dependencies

- **Requires:** Issue #1 (IoFrame ref-counting) — completion queue doesn't need ref-counting, but without it the digipeater can't hold frames across TX.
- **Blocks:** Issue #3 (KISS ACK Mode) — needs TX confirmation to send ACK/NAK.
- **Can be implemented independently** of libaprsroute integration.

## Risks

- **Stack depth:** `osMessagePut` called from `process()` adds ~100 bytes to Encoder::process() stack. The FreeRTOS modulator task stack is 512 words (2KB). Check current usage with `.su` files.
- **Race:** Completion queue is FreeRTOS queue (ISR-safe). Modulator and digipeater run in different tasks. Queue provides the synchronization.

---

## Estimated Time

- Investigation (tasks 1-4): 30 min
- Design decision: 15 min
- Implementation (tasks 5-11): 2 hours
- Documentation (task 12): 15 min
- **Total: ~3 hours** plus hardware testing
