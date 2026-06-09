# TX Confirmation — Implementation Plan (#2)

> **For Hermes:** Delegate to OpenCode with MiMo model to implement task-by-task.

**Goal:** Add delivery confirmation from the modulator back to TX callers by storing an optional completion queue handle on each IoFrame. When transmission completes (or fails), the modulator posts the frame pointer back to the caller's queue. The caller checks `frame->tx_result()` to determine outcome.

**Architecture:** Add `osMessageQId tx_completion_queue_` (4 bytes) and `TxResult tx_result_` (1 byte) to the Frame template. In `hdlc::Encoder::process()`, set the result and notify before releasing. Zero overhead for callers that don't set a completion queue. 192 bytes total for the 48-frame pool.

**Tech Stack:** C++20, STM32L4P5RETx (ARM Cortex-M4F), FreeRTOS CMSIS-RTOS v1, Boost.Intrusive

**Design decisions:**
- **Completion queue on Frame** (not callback, not task notify) — caller sets queue handle, modulator posts result
- **Frame IS the notification** — modulator posts the IoFrame* back; caller checks `tx_result()` on receipt
- **`osMessageQId` field** — 0 means "no notification" (existing behavior for all current callers)
- **tx_completion_queue_ resets in `clear()`** — re-acquired frames start with no completion queue
- **M17 parity** — M17Encoder gets the same treatment for consistency

---

## Before starting

- Branch from master: `git checkout -b feature/tx-confirmation`
- Read: `Core/TNC/HdlcFrame.hpp`, `Core/TNC/HdlcFrame.cpp`, `Core/TNC/HDLCEncoder.hpp`
- Read: `Core/TNC/M17Encoder.h`, `Core/TNC/Encoder.h`, `Core/TNC/Digipeater.cpp`

---

### Task 1: Add TxResult enum to HdlcFrame.hpp

**Objective:** Define the transmission result types used by the modulator to report outcome.

**Files:**
- Modify: `Core/TNC/HdlcFrame.hpp`

**Step 1: Add TxResult enum before the Frame class**

In `Core/TNC/HdlcFrame.hpp`, inside `namespace mobilinkd { namespace tnc { namespace hdlc {`, before the `Frame` template class (before line 28):

```cpp
enum class TxResult : uint8_t
{
    NONE = 0,           // Not transmitted yet
    SENT = 1,           // Frame transmitted successfully
    CSMA_TIMEOUT = 2,   // CSMA timed out, frame dropped
    ABORTED = 3,        // Transmission aborted (DAC underrun, etc.)
};
```

**Step 2: Build verification**

```bash
cd /home/hermes/.hermes/profiles/kyle/workspace/tnc4-firmware
cmake --build build/Debug --target tnc4-firmware 2>&1 | tail -10
```

Expected: Successful compilation.

**Step 3: Commit**

```bash
git add Core/TNC/HdlcFrame.hpp
git commit -m "feat: add TxResult enum for modulator delivery confirmation"
```

---

### Task 2: Add completion queue and result fields to Frame

**Objective:** Add the `tx_completion_queue_` and `tx_result_` members to the Frame template, with accessors and reset in `clear()`.

**Files:**
- Modify: `Core/TNC/HdlcFrame.hpp`

**Step 1: Add private members**

In the Frame class, private section (after `frame_type_`, around line 47):

```cpp
    osMessageQId tx_completion_queue_{0};
    TxResult tx_result_{TxResult::NONE};
```

**Step 2: Add public accessors**

In the Frame class public section (after `source()`, around line 87):

```cpp
    osMessageQId tx_completion_queue() const { return tx_completion_queue_; }
    void tx_completion_queue(osMessageQId q) { tx_completion_queue_ = q; }

    TxResult tx_result() const { return tx_result_; }
    void tx_result(TxResult r) { tx_result_ = r; }
```

**Step 3: Update clear() to reset both**

Currently at line 89-95. Modify to add:

```cpp
    void clear() {
        data_.clear();
        crc_ = -1;
        fcs_ = -2;
        complete_ = false;
        frame_type_ = 0;
        ref_count_ = 1;
        tx_completion_queue_ = 0;
        tx_result_ = TxResult::NONE;
    }
```

**Step 4: Build verification**

```bash
cmake --build build/Debug --target tnc4-firmware 2>&1 | tail -10
```

Expected: Successful compilation.

**Step 5: Commit**

```bash
git add Core/TNC/HdlcFrame.hpp
git commit -m "feat: add TX completion queue and result fields to Frame"
```

---

### Task 3: Add completion notification to hdlc::Encoder::process()

**Objective:** In `hdlc::Encoder::process()`, set `TxResult` on the frame and post it back to the caller's completion queue before releasing. Also add a private `notify_completion()` helper.

**Files:**
- Modify: `Core/TNC/HDLCEncoder.hpp`

**Step 1: Add notify_completion() private method**

In the `Encoder` struct, private section, add:

```cpp
    void notify_completion(IoFrame* frame, TxResult result) {
        auto q = frame->tx_completion_queue();
        if (q) {
            frame->tx_result(result);
            osMessagePut(q, reinterpret_cast<uint32_t>(frame), 0);
        }
    }
```

**Step 2: Add notification in process() — CSMA timeout path**

In `process()`, the CSMA timeout path currently (around line 206-208):

```cpp
            if (not do_csma()) {
                release(frame);
                return;
            }
```

Change to:

```cpp
            if (not do_csma()) {
                notify_completion(frame, TxResult::CSMA_TIMEOUT);
                release(frame);
                return;
            }
```

**Step 3: Add notification in process() — successful TX path**

At the end of `process()` (around line 219-221):

```cpp
        for (auto c : *frame) send(c);
        release(frame);
        send_tail();
```

Change to:

```cpp
        for (auto c : *frame) send(c);
        notify_completion(frame, TxResult::SENT);
        release(frame);
        send_tail();
```

**Step 4: Build verification**

```bash
cmake --build build/Debug --target tnc4-firmware 2>&1 | tail -10
```

Expected: Successful compilation.

**Step 5: Commit**

```bash
git add Core/TNC/HDLCEncoder.hpp
git commit -m "feat: add TX completion notification to hdlc::Encoder::process()"
```

---

### Task 4: Add completion notification to M17Encoder

**Objective:** Mirror the completion notification pattern in M17Encoder for consistency. Every code path in M17Encoder that calls `release()` after TX must notify first.

**Files:**
- Modify: `Core/TNC/M17Encoder.h`

**Step 1: Read M17Encoder to find all release() call sites after TX**

```bash
grep -n "release" Core/TNC/M17Encoder.h
```

**Step 2: Add notify_completion() private method**

In the `M17Encoder` struct, add the same helper:

```cpp
    void notify_completion(IoFrame* frame, TxResult result) {
        auto q = frame->tx_completion_queue();
        if (q) {
            frame->tx_result(result);
            osMessagePut(q, reinterpret_cast<uint32_t>(frame), 0);
        }
    }
```

**Step 3: Add notification before each release() after TX**

For each `release(frame)` call after transmission:
- Before: `release(frame);`
- After: `notify_completion(frame, TxResult::SENT); release(frame);`

For CSMA timeout / drop paths (if any):
- Before: `release(frame);`
- After: `notify_completion(frame, TxResult::CSMA_TIMEOUT); release(frame);`

**Step 4: Build verification**

```bash
cmake --build build/Debug --target tnc4-firmware 2>&1 | tail -10
```

Expected: Successful compilation.

**Step 5: Commit**

```bash
git add Core/TNC/M17Encoder.h
git commit -m "feat: add TX completion notification to M17Encoder"
```

---

### Task 5: Wire up digipeater completion handling

**Objective:** In the digipeater stub, set the completion queue on frames before enqueuing to the modulator, and handle completion notifications in the digipeater task loop. This validates the mechanism end-to-end.

**Files:**
- Modify: `Core/TNC/Digipeater.cpp`

**Step 1: After rewriting a frame for digipeating, set the completion queue**

In the `startDigipeaterTask` function, after the frame is rewritten (currently commented out at line 49), add:

```cpp
    auto digi_frame = digi->rewrite_frame(frame);
    hdlc::add_ref(digi_frame);  // Digipeater holds a reference
    digi_frame->tx_completion_queue(digipeaterQueueHandle);
    osMessagePut(hdlcOutputQueueHandle,
        reinterpret_cast<uint32_t>(digi_frame),
        osWaitForever);
```

**Step 2: Handle TxConfirm messages in the task loop**

In the `for(;;)` loop, after the command check, add handling for completion notifications:

```cpp
    // Check if this is a TX completion notification
    auto frame = static_cast<IoFrame*>(evt.value.p);
    if (frame->tx_result() != hdlc::TxResult::NONE)
    {
        // Frame came back from modulator — TX is complete.
        // tx_result() is SENT, CSMA_TIMEOUT, or ABORTED.
        hdlc::release(frame);  // Release digipeater's reference
        continue;
    }
```

Note: On initial receipt from the digipeaterQueue, tx_result() is NONE (the frame hasn't been through the modulator yet). After the modulator processes and posts it back, tx_result() will be non-NONE.

**Step 3: Build verification**

```bash
cmake --build build/Debug --target tnc4-firmware 2>&1 | tail -10
```

Expected: Successful compilation.

**Step 4: Commit**

```bash
git add Core/TNC/Digipeater.cpp
git commit -m "feat: wire digipeater TX completion handling"
```

---

### Task 6: Full build and size check

**Objective:** Verify the firmware compiles cleanly and confirm the memory overhead.

**Step 1: Clean rebuild**

```bash
rm -rf build/Debug
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -S . -B build/Debug -G Ninja
cmake --build build/Debug --target tnc4-firmware 2>&1
```

Expected: Zero errors, zero new warnings.

**Step 2: Size check**

```bash
arm-none-eabi-size build/Debug/tnc4-firmware.elf
```

Expected: Text/data/bss sizes. The new fields add 5 bytes per IoFrame (4 for queue handle + 1 for TxResult) × 48 frames = 240 bytes to `.bss`. Negligible on STM32L4P5 (512KB SRAM).

**Step 3: Commit**

No commit needed — verification only.

---

## Verification

After all tasks complete:

1. `git log --oneline feature/tx-confirmation` — should show 5 commits
2. All existing callers (IOEventTask, SerialPort, UsbPort, BERT) continue to work unchanged — their frames have `tx_completion_queue_ == 0`, no notification occurs
3. Digipeater sets completion queue, receives notification, releases reference
4. Frame pool capacity of 48 frames is unchanged
5. M17Encoder has parity with HDLCEncoder

## What this enables

- Digipeater can track when its digipeated frames complete transmission
- KISS ACK Mode (issue #3) can build on this to send ACK/NAK
- TX statistics can be accumulated from completion notifications

## Files changed

| File | Change |
|------|--------|
| `Core/TNC/HdlcFrame.hpp` | Add TxResult enum, tx_completion_queue_, tx_result_ fields and accessors, reset in clear() |
| `Core/TNC/HDLCEncoder.hpp` | Add notify_completion(), call before release() in process() |
| `Core/TNC/M17Encoder.h` | Add notify_completion(), call before release() |
| `Core/TNC/Digipeater.cpp` | Set completion queue, handle completion notifications |

## Risks

- **Low:** 240-byte overhead is negligible on 512KB SRAM
- **Low:** Existing callers unchanged — `tx_completion_queue_ == 0` means no notification
- **Medium (deferred):** `notify_completion()` calls `osMessagePut` with timeout 0 — if the receiver's queue is full, the notification is silently dropped. The receiver must drain its queue faster than the modulator produces frames. For the digipeater, this is fine — it processes frames one at a time.
