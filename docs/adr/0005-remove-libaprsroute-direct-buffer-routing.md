# 0005: Remove libaprsroute — direct AX.25 buffer routing

**Status:** Proposed
**Date:** 2026-07-29

## Context

The digipeater integration plan (2026-06-03) called for libaprsroute as a git submodule for APRS path routing. The library was integrated in commit `00fbf96` and immediately removed in commit `8b25d28`.

libaprsroute is an MIT-licensed, header-only C++20 APRS routing library (~4,700 lines) by Ion Todirel. It provides `try_route_packet()` for path manipulation: preempt, substitute, skip-complete, n-N hop decrement. The API is string-based — addresses are `std::string_view` or `std::array<char, 10>`, packets are parsed from ASCII representations like `"N0CALL>APRS,WIDE1-1,WIDE2-2:data"`.

## Problem

The string-based API created a redundant encode-decode-encode pipeline in `rewrite_frame()`:

```
AX.25 7-byte address blocks in linear_buf_
  → parse_ax25_frame(): right-shift bytes, trim spaces, snprintf("CALL-7")
    → libaprsroute try_route_packet(): parse strings back to address objects
      → returns std::array<std::array<char, 10>, 8>
        → encode_addr(): find '-', find '*', atoi(), shift left
          → back to 7-byte address blocks
```

Four format conversions for addresses that start and end as 7-byte blocks. The `parse_ax25_frame()` function (90 lines) and `encode_addr()` lambda (30 lines) existed solely to bridge between AX.25 buffer bytes and libaprsroute's string API.

Meanwhile, `can_repeat()` already performed alias matching directly on buffer bytes using `match_shifted_callsign()` — zero string conversion, zero external library. The routing logic for an n-N digipeater is simple enough for direct byte operations:

- Match address against alias: compare shifted bytes
- Read hop count: `(buffer[pos + 6] >> 1) & 0x0F`
- Decrement hop: set SSID byte with new value, H-bit, and C-bit

## Decision

Remove libaprsroute. Implement `rewrite_frame()` with direct AX.25 address block manipulation in the 330-byte `linear_buf_`. No external routing library. No string conversion on the packet processing path.

## Consequences

**Positive:**
- Eliminates 120 lines of string-format bridge code (two format conversions eliminated entirely)
- Eliminates an external dependency (git submodule, include path, compile definitions)
- Eliminates `std::string_view` usage in embedded code (libaprsroute uses it heavily, though safely without heap)
- No `snprintf`, `atoi`, `strchr` calls on the digipeater hot path
- Consistent approach: `can_repeat()` and `rewrite_frame()` both operate on raw bytes, sharing the same `match_shifted_callsign()` helper
- libaprsroute's `route_state` and `routing_state` structs used ~200 bytes of digipeater task stack — eliminated

**Negative:**
- We lost libaprsroute's 256-route test suite (routes.json + tests.cpp). Zero digipeater tests exist in the firmware. See ADR-0006 for the test porting plan.
- The MIT license requires attribution in "all copies or substantial portions." The replacement code implements the same routing modes (PREEMPT_FRONT, PREEMPT_TRUNCATE, PREEMPT_DROP, PREEMPT_MARK, SUBSTITUTE, SKIP_COMPLETE) and the same n-N routing algorithm. Attribution is required. See ATTRIBUTION.md.
- The routing mode flag design was informed by libaprsroute's `routing_option` enum. This architectural influence requires documentation.
- Unimplemented routing modes (preempt for explicit routing, traceless routing, excessive hop trapping) must be implemented from scratch rather than inherited from the library.

## Attribution

libaprsroute by Ion Todirel (MIT License, copyright 2024-2025) served as the reference implementation for APRS digipeater routing. The routing mode flags (PREEMPT_FRONT, PREEMPT_TRUNCATE, PREEMPT_DROP, PREEMPT_MARK, SUBSTITUTE, SKIP_COMPLETE) and the n-N routing algorithm were informed by studying libaprsroute's design. The replacement implementation operates at a different level of abstraction (raw AX.25 bytes vs. parsed string representations) but the routing semantics were validated against libaprsroute's test vectors. See ATTRIBUTION.md for the full license text and copyright.

## Test porting plan

libaprsroute ships 6147 lines of test infrastructure including 256 routing test cases in routes.json. These test vectors define expected behavior for preempt, n-N routing, explicit routing, edge cases, and error conditions. ADR-0006 will detail the plan to port these test vectors into a host-side test harness for `can_repeat()` and `rewrite_frame()`. Until then, the digipeater has zero automated tests.
