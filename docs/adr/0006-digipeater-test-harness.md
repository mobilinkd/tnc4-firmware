# ADR-0006: Port libaprsroute test vectors to host-side digipeater test harness

**Status:** Proposed
**Date:** 2026-07-29

## Context

The TNC4 digipeater (`Core/TNC/Digipeater.hpp`, `Core/TNC/Digipeater.cpp`) has zero automated tests. `can_repeat()` and `rewrite_frame()` are pure functions operating on byte buffers — they should be tested off-target without hardware.

libaprsroute ships 256 routing test cases in `routes.json` covering preempt, n-N routing, explicit routing, edge cases, and error conditions. These test vectors are the reference for APRS digipeater behavior.

## Test vector breakdown

| Category | Count | Currently implemented? |
|----------|-------|----------------------|
| n-N routing: WIDE/TRACE/RELAY hop decrement | 110 | Yes |
| Explicit: preempt_front | 13 | No |
| Explicit: preempt_truncate | 18 | No |
| Explicit: preempt_drop | 8 | No |
| Explicit: preempt_mark | 1 | No |
| Explicit: substitute_explicit_address | 21 | No |
| Explicit: combined modes (e.g. preempt_front+substitute) | 18 | No |
| n-N: substitute_complete_n_N_address | 26 | Yes |
| n-N: skip_complete_n_N_address | 4 | Yes |
| n-N: traceless | 14 | No |
| n-N: reject/trap limit-exceeding | 8 | No |
| route_self / strict | 3 | No |
| Malformed packets (no colon, no >) | 6 | Edge cases |
| Duplicate packet rejection | 6 | N/A (dedupe is separate) |

## What we can test NOW

The current implementation handles n-N routing with `can_repeat()` matching WIDE/TRACE/RELAY/ECHO/GATE prefixes and `rewrite_frame()` doing SSID hop decrement, H-bit marking, and C-bit management. Substitution of exhausted n-N aliases is hardcoded (always on). The `ROUTING_SKIP_COMPLETE` flag is read from settings.

**~140 testable cases** from the 256:
- 110 n-N routing tests (WIDE1-1, WIDE2-2 hop decrement and H-bit/C-bit)
- 26 substitute_complete_n_N_address tests
- 4 skip_complete_n_N_address tests

**~116 cases require unimplemented routing modes** (preempt, explicit substitution, traceless, trap/reject). These should be added as DISABLED tests or TODO markers.

## Implementation plan

Three phases, two parallel subagents in phase 2.

### Phase 1: Test harness (subagent A)

Deliverable: `tests/test_digipeater_harness.hpp` + `tests/CMakeLists.txt` host build target.

Tasks:
1. Create `tests/test_digipeater_harness.hpp` with:
   - `parse_ax25_frame(const char* packet)` → `LinearBuffer` (ASCII to 7-byte blocks)
   - `ax25_frame_to_string(const LinearBuffer&)` → ASCII string (for assertion messages)
   - `make_alias(const char* call, uint8_t hops, bool set, bool use)` → `kiss::Alias`
   - `make_hardware()` → `kiss::Hardware` with digipeater enabled, routing mode set
   - Stub `HAL_GetTick()` returning controllable time for dedupe window tests
   - Stub `osMessagePut()` / `osMessageGet()` for queue tests
2. Create `tests/CMakeLists.txt` with host-side build target:
   - Include path: `../Core/TNC`, `../Core`, `../Drivers/CMSIS/Include`
   - Compile definitions: `TNC4_HOST_BUILD`, `KISS_LOGGING`
   - Link: nothing (header-only digipeater)
3. Write 5 smoke tests:
   - parse_ax25_frame roundtrip: "N0CALL>APRS,WIDE1-1:data" → bytes → string
   - can_repeat matches WIDE1-1 alias against WIDE1-1 path
   - can_repeat rejects WIDE2-2 when alias is WIDE1-1
   - rewrite_frame decrements WIDE2-2 → WIDE2-1 with H-bit set
   - rewrite_frame marks WIDE2-1 complete (H-bit + C-bit)
4. Verify: `cd build && cmake .. && cmake --build . --target test_digipeater && ./test_digipeater`
5. Commit as `test: add host-side digipeater test harness`

### Phase 2: n-N routing tests + explicit routing tests (subagents B and C, parallel)

**Subagent B: n-N routing test suite**
Deliverable: `tests/test_digipeater_nn.cpp` (~110 test cases)

Tasks:
1. Include `test_digipeater_harness.hpp`
2. Port all 110 n-N routing test cases from `routes.json`
3. For each test case:
   - Parse `original_packet` → AX.25 buffer
   - Configure alias matching the test's `address` field
   - Call `can_repeat()` → verify `routed` matches expected
   - If routed=true: call `rewrite_frame()` → verify output matches `routed_packet`
4. Add edge cases from our codebase:
   - Path with 8 addresses (full buffer)
   - SSID=0 rejects
   - SSID beyond alias hops rejects
   - NUL-padded callsigns
5. Verify all 110 pass
6. Commit as `test: n-N routing test suite (110 cases from libaprsroute)`

**Subagent B test cases will verify these specific behaviors:**
- WIDE1-1 → WIDE1* (decrement from 1 to 0, mark complete)
- WIDE1-2 → DIGI*,WIDE1-1 (decrement, insert callsign before)
- WIDE2-2 → DIGI*,WIDE2-1 (same for WIDE2)
- WIDE2-1 → WIDE2* (last hop, mark complete)
- WIDE2-0 → not routed (SSID already 0)
- WIDE1 without dash → not routed (no SSID to decrement)
- WIDE2-3 → DIGI*,WIDE2-2 (multi-hop remaining)
- TRACE2-2, RELAY-1, ECHO2-1 same patterns
- WIDE1-2,DIGI → not routed (our callsign in path before alias)
- WIDE*,WIDE → not routed (already marked used)
- 7 used addresses + WIDE2-2 → WIDE2-1 (path full, no insertion)

**Subagent C: Explicit routing tests (currently implemented modes)**
Deliverable: `tests/test_digipeater_explicit.cpp` (~30 test cases for implemented modes, ~116 DISABLED for future)

Tasks:
1. Include `test_digipeater_harness.hpp`
2. Test currently-implemented explicit routing:
   - Exact callsign match against alias with `set=true, use=true`
   - Callsign match with H-bit already set (not routed — already used)
   - Callsign match with C-bit set (not routed — last address, no more hops)
   - Our callsign in destination field (not routed — APRS rule)
   - Our callsign in source field (not routed — APRS rule)
3. Test substitute_complete_n_N_address:
   - WIDE1-1 → DIGI* (replace aliased address with our callsign)
   - WIDE2-1 → DIGI* (same, with correct C-bit on DIGI)
4. Test skip_complete_n_N_address:
   - WIDE1*,WIDE2-2 → DIGI*,WIDE2-2 (skip WIDE1*, process WIDE2-2)
   - WIDE2-2*,WIDE1-1 → WIDE2-2*,DIGI* (past WIDE2*, process WIDE1-1)
5. Add DISABLED test stubs for unimplemented modes:
   - `DISABLED_preempt_front` — 13 cases
   - `DISABLED_preempt_truncate` — 18 cases
   - `DISABLED_preempt_drop` — 8 cases
   - `DISABLED_preempt_mark` — 1 case
   - `DISABLED_substitute_explicit_address` — 21 cases
   - `DISABLED_traceless` — 14 cases
   - `DISABLED_trap_reject` — 8 cases
6. Verify all implemented cases pass, DISABLED cases compile
7. Commit as `test: explicit routing test suite with TODO markers for preempt modes`

### Phase 3: Integration

1. Build all three test targets from a single `cmake --build .`
2. Run full suite: `ctest` or manual `./test_digipeater_harness && ./test_digipeater_nn && ./test_digipeater_explicit`
3. Fix any test failures
4. Commit any fixes
5. Update HANDOFF.md with test status

## Test harness API design

```cpp
// tests/test_digipeater_harness.hpp

struct LinearBuffer {
    std::array<uint8_t, 330> data{};
    size_t size = 0;
};

struct TestFrame {
    std::string dest;    // "N0CALL"
    std::string src;     // "APRS"
    struct PathAddr {
        std::string call;
        uint8_t ssid = 0;
        bool h_bit = false;
        bool c_bit = false;
    };
    std::vector<PathAddr> path;
    std::string info;    // "data"
};

// Parse ASCII APRS packet to AX.25 7-byte blocks
LinearBuffer parse_ax25_frame(const std::string& packet);

// Parse to structured form for assertions
TestFrame parse_test_frame(const std::string& packet);

// Convert AX.25 buffer back to ASCII for debug output
std::string ax25_to_string(const LinearBuffer& buf);

// Create an alias
kiss::Alias make_alias(const std::string& call, uint8_t hops, bool set, bool use);

// Create hardware settings with digipeater enabled
kiss::Hardware make_test_hardware(const std::string& mycall);

// Set routing mode flags
void set_routing_mode(kiss::Hardware& hw, uint8_t mode);
```

## Risks

1. **CMSIS dependency**: `Digipeater.hpp` includes `<cmsis_os.h>` for `osMessageQId` and `osMessagePut`/`osMessageGet`. These need stubs for host-side compilation. The digipeater task loop (`for(;;) { osMessageGet(...) }`) is not part of the test — we test `can_repeat()` and `rewrite_frame()` directly.

2. **HAL_GetTick dependency**: Deduplication uses `HAL_GetTick()` for 30-second timestamp windows. Stub it to return a controllable value.

3. **EEPROM mock**: `kiss::settings()` returns a reference to the global Hardware struct. The harness needs to either provide a writable instance or mock the function.

4. **Build system**: The host-side test binary uses the same source files as the embedded target. Need `#ifndef TNC4_HOST_BUILD` guards around hardware-specific includes.

5. **Test fidelity**: The routes.json tests were designed for libaprsroute's string-based API. Our byte-level implementation may produce slightly different output for edge cases (trailing spaces in callsigns, SSID encoding in byte 6). Tests must assert on the semantic result (routed=true/false, path modified correctly) not on exact byte-for-byte output.

## Test example

```cpp
TEST(DigipeaterNN, Wide1_1_SingleHop_DecToZero) {
    // "N0CALL>APRS,WIDE1-1:data" routed by WIDE1 alias with hops=1
    auto buf = parse_ax25_frame("N0CALL>APRS,WIDE1-1:data");
    
    kiss::Alias aliases[] = {make_alias("WIDE1", 1, true, true)};
    auto hw = make_test_hardware("DIGI");
    // Substitution of exhausted n-N aliases is hardcoded (always on).
    
    mobilinkd::tnc::Digipeater digi(aliases, nullptr);
    
    EXPECT_TRUE(digi.can_repeat(buf.data.data(), buf.size));
    
    auto result = digi.rewrite_frame(buf.data.data(), buf.size);
    ASSERT_TRUE(result.has_value());
    
    auto out = ax25_to_string(*result);
    // WIDE1-1 becomes WIDE1* (H-bit set, SSID=0, marked complete)
    EXPECT_TRUE(out.find("WIDE1*") != std::string::npos);
}
```
