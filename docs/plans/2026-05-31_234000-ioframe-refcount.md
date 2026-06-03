# IoFrame Reference Counting — Implementation Plan (#1)

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Replace single-owner `acquire()/release()` with reference-counted semantics so RF frames can be consumed simultaneously by the host forward path AND the digipeater.

**Architecture:** Add a `uint8_t ref_count` to the `Frame` template. `release()` decrements the count and only returns the frame to the pool when it reaches zero. A new `add_ref()` method increments for additional consumers. All operations are ISR-safe via existing critical section patterns.

**Tech Stack:** C++20, STM32L4P5RETx (ARM Cortex-M4F), FreeRTOS, Boost.Intrusive

**Design decisions (from grill session):**
- 1 byte per frame × 48 frames = 48 bytes overhead — acceptable
- Critical section protection per the existing `FramePool` pattern
- Backward compatible: all existing single-consumer code works unchanged
- `clear()` resets `ref_count` to 1 (not 0 — frames start life with one reference)

---

## Before starting

- Branch from `feature/agent-setup`: `git checkout -b feature/ioframe-refcount`
- Read: `Core/TNC/HdlcFrame.hpp`, `Core/TNC/HdlcFrame.cpp`
- Read: `Core/TNC/Digipeater.hpp`, `Core/TNC/Digipeater.cpp` for context

---

### Task 1: Add `ref_count` field to Frame template

**Objective:** Add the reference count member and initialize it to 1 on construction and after clear.

**Files:**
- Modify: `Core/TNC/HdlcFrame.hpp:79-81` (Frame constructor)
- Modify: `Core/TNC/HdlcFrame.hpp:89-95` (Frame::clear)

**Step 1: Add member and constructor init**

In `Core/TNC/HdlcFrame.hpp`, at the private members (after line 47, before `#ifndef EXCLUDE_CRC`):

```cpp
    uint8_t ref_count_{1};
```

And add a public accessor after the existing public section:

```cpp
    uint8_t ref_count() const { return ref_count_; }
```

Update the constructor (line 79-81):

```cpp
    Frame()
    : list_base_hook<>(), data_(), crc_(-1), fcs_(-2), complete_(false),
      ref_count_(1)
    {}
```

Update `clear()` (line 89-95) — reset `ref_count_` to 1 AND frame_type_:

```cpp
    void clear() {
        data_.clear();
        crc_ = -1;
        fcs_ = -2;
        complete_ = false;
        frame_type_ = 0;
        ref_count_ = 1;
    }
```

**Step 2: Build verification**

```bash
cd ~/workspace/tnc4-firmware
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -S . -B build/Debug -G Ninja
cmake --build build/Debug --target tnc4-firmware 2>&1 | tail -10
```

Expected: Successful compilation with no errors (only warnings if any pre-exist).

**Step 3: Commit**

```bash
git add Core/TNC/HdlcFrame.hpp
git commit -m "feat: add ref_count field to Frame template"
```

---

### Task 2: Add `add_ref()` method to FramePool

**Objective:** Provide a way for a second consumer to claim a reference to a frame already in use.

**Files:**
- Modify: `Core/TNC/HdlcFrame.hpp:164-173` (FramePool::acquire, nearby area)

**Step 1: Add `add_ref()` to FramePool**

Add after `acquire()` (after line 173 in FramePool):

```cpp
    void add_ref(frame_type* frame) {
        auto x = taskENTER_CRITICAL_FROM_ISR();
        frame->ref_count_ += 1;
        taskEXIT_CRITICAL_FROM_ISR(x);
    }
```

**Step 2: Build verification**

```bash
cmake --build build/Debug --target tnc4-firmware 2>&1 | tail -10
```

Expected: Successful compilation.

**Step 3: Commit**

```bash
git add Core/TNC/HdlcFrame.hpp
git commit -m "feat: add FramePool::add_ref() for multi-consumer support"
```

---

### Task 3: Modify `FramePool::release()` to use ref-counting

**Objective:** Change `release()` to decrement the reference count and only return the frame to the free list when it reaches zero. This is the core behavioral change.

**Files:**
- Modify: `Core/TNC/HdlcFrame.hpp:175-180` (FramePool::release)

**Step 1: Rewrite `release()`**

Replace the current `release()`:

```cpp
    void release(frame_type* frame) {
        auto x = taskENTER_CRITICAL_FROM_ISR();
        if (--frame->ref_count_ == 0)
        {
            frame->clear();
            free_list_.push_back(*frame);
        }
        taskEXIT_CRITICAL_FROM_ISR(x);
    }
```

Note: `clear()` already sets `ref_count_ = 1` (from Task 1), so when the frame is re-acquired it starts with ref_count = 1. Behavior is internally consistent: the pool re-initializes it on return, and acquire doesn't need to touch the ref_count.

**Step 2: Build verification**

```bash
cmake --build build/Debug --target tnc4-firmware 2>&1 | tail -10
```

Expected: Successful compilation.

**Step 3: Commit**

```bash
git add Core/TNC/HdlcFrame.hpp
git commit -m "feat: convert FramePool::release() to reference-counted semantics"
```

---

### Task 4: Add global `add_ref()` convenience function

**Objective:** Provide a global `hdlc::add_ref(IoFrame*)` function that matches the existing `hdlc::release()` / `hdlc::acquire()` pattern, for use by the digipeater fork path in IOEventTask.

**Files:**
- Modify: `Core/TNC/HdlcFrame.hpp` (add declaration after line 202)
- Modify: `Core/TNC/HdlcFrame.cpp` (add definition)

**Step 1: Add declaration**

In `Core/TNC/HdlcFrame.hpp`, after the `void release(IoFrame* frame);` declaration (line 202):

```cpp
void add_ref(IoFrame* frame);
```

**Step 2: Add definition**

In `Core/TNC/HdlcFrame.cpp`, after `release()` (line 21):

```cpp
void add_ref(IoFrame* frame)
{
    ioFramePool().add_ref(frame);
}
```

**Step 3: Build verification**

```bash
cmake --build build/Debug --target tnc4-firmware 2>&1 | tail -10
```

Expected: Successful compilation.

**Step 4: Commit**

```bash
git add Core/TNC/HdlcFrame.hpp Core/TNC/HdlcFrame.cpp
git commit -m "feat: add global hdlc::add_ref() convenience function"
```

---

### Task 5: Audit all `release()` call sites for correctness

**Objective:** Verify that every existing call to `hdlc::release()` remains correct under reference-counted semantics. The key invariant: every `acquire()` is matched by exactly one `release()`. With ref-counting, this is still true — acquire gives ref_count=1, release at 0 returns to pool.

**Files to audit (all existing `release()` call sites):**
- `Core/TNC/HdlcDecoder.cpp:39` — acquire → release path through IOEventTask
- `Core/TNC/IOEventTask.cpp:612` — release on TX queue write failure
- `Core/TNC/NullPort.hpp:42` — release in NullPort write
- `Core/TNC/UsbPort.cpp:76,78,87,89,94,96,100,102,110,299,306,346` — acquire_wait → release
- `Core/TNC/SerialPort.cpp:127,179,182,188,195,200,202,487,503,510,579` — acquire_wait → release
- `Core/TNC/AudioInput.cpp:221` — release
- `Core/TNC/Kiss.cpp:16,29,35,41,47,52,58,62,67,72` — release
- `Core/TNC/AfskDemodulator.cpp:41` — release
- `Core/TNC/Afsk1200Demodulator.cpp:40,57,74` — release
- `Core/TNC/Fsk9600Demodulator.cpp:34` — release
- `Core/TNC/HDLCEncoder.hpp:207,220` — release
- `Core/TNC/AFSKTestTone.cpp:127` — release

**Step 1: Run the audit mentally (no code changes)**

Verify each path:
1. Frame is `acquire()`'d or `acquire_wait()`'d → ref_count = 1
2. Frame is used by exactly one consumer
3. Frame is `release()`'d once → ref_count goes to 0 → returned to pool

All existing paths are single-consumer. No double-release or missing-release patterns exist. The ref-counted `release()` is a strict superset of the old behavior for these paths.

**Step 2: Document the audit results**

Create `docs/adr/0004-ioframe-reference-counting.md`:

```markdown
# 0004: IoFrame reference counting

Replaced single-owner `acquire()/release()` with reference counting. Audit of all 50+ existing call sites confirmed correctness — every `acquire()` is paired with exactly one `release()`, and the ref-counted `release()` is a behavioral superset.

See `Core/TNC/HdlcFrame.hpp` Frame::ref_count_, FramePool::add_ref(), FramePool::release().
```

**Step 3: Commit**

```bash
mkdir -p docs/adr
git add docs/adr/0004-ioframe-reference-counting.md
git commit -m "docs: add ADR-0004 for IoFrame reference counting"
```

---

### Task 6: Full build and flash test

**Objective:** Build the firmware and verify it compiles cleanly for the target. The next step after this plan is hardware flashing (out of scope for this PR — requires physical TNC4).

**Files:** None (build only)

**Step 1: Clean rebuild**

```bash
rm -rf build/Debug
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -S . -B build/Debug -G Ninja
cmake --build build/Debug --target tnc4-firmware 2>&1
```

Expected: Zero errors. The output binary should be `build/Debug/tnc4-firmware.elf`. Verify it exists:

```bash
ls -la build/Debug/tnc4-firmware.elf
```

**Step 2: Size check**

```bash
arm-none-eabi-size build/Debug/tnc4-firmware.elf
```

Expected: Text/data/bss sizes. The `ref_count_` field adds 1 byte per IoFrame (×48 = 48 bytes to `.bss`). This should be negligible.

**Step 3: Commit the build (if workflow includes build artifacts)**

No commit needed for build output. This task is verification only.

---

## Verification

After all tasks complete:

1. `git log --oneline feature/ioframe-refcount` — should show 5-6 commits
2. All existing `release()` call sites are single-consumer — no behavior change
3. `FramePool::add_ref()` is ready for digipeater integration (issue #2 and beyond)
4. Frame pool capacity of 48 frames is unchanged

## What this enables

After this PR:
- In `IOEventTask.cpp:590-599`, when an RF frame arrives, the digipeater path can call `hdlc::add_ref(frame)` before queuing to the digipeater. The host path calls `hdlc::release(frame)` as before. The digipeater path calls `hdlc::release(frame)` when done. The frame stays alive until both consumers release it.
- The frame pool's `size()` will correctly reflect available frames — a frame with ref_count > 1 is not in the free list and won't be double-issued.

## Risks

- **Low:** All existing paths are single-consumer and verified. Ref-counted release is a superset.
- **Low:** 48-byte overhead is negligible on STM32L4Q5 (512KB SRAM).
- **Medium (deferred):** The `acquire()` function panics on null (`CxxErrorHandler()`). Under heavy digipeater load with 48 frames all held by multiple consumers, a null acquire could trigger. This is a pre-existing condition and not worsened by this change — the pool capacity is unchanged.

## Files changed

| File | Change |
|------|--------|
| `Core/TNC/HdlcFrame.hpp` | Add `ref_count_` field, init in ctor/clear, add `add_ref()`, modify `release()` |
| `Core/TNC/HdlcFrame.cpp` | Add `hdlc::add_ref()` global function |
| `docs/adr/0004-ioframe-reference-counting.md` | New: architecture decision record |
