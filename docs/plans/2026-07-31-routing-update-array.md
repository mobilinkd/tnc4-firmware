# Routing Update-Array Redesign

Date: 2026-07-31
Branch: feature/digipeater-integration
Status: PLAN -- pending Rob's review. No code changed.

## TL;DR

Replace the ad-hoc scan+rewrite logic in DigipeaterCore with an explicit
per-path-location update array.  The scan walks the path left to right and
writes a bitmask per location describing exactly what the rewrite will do.
The rewrite executes the array mechanically.  This adopts the reference's
"route only on the first unused address" blocking model and the broken-digi
exhausted-alias correction, and removes the buggy address-removing
skip_complete block.

H-bits are NEVER unset.  Every used address keeps its H-bit on the wire.
The reference test vectors use the TNC-2 display convention (only the last
used digi shows `*`); our wire output keeps all H-bits set.  Expected
outputs in this document use our wire format (all H-bits shown as `*`).

## Decisions locked

1. Route only on the first unused address after the used prefix (reference
   model).  A stray unused address that does not match any of our aliases
   blocks the route -- the packet is dropped.  This changes current behavior
   (we currently route on the first matching alias regardless of position).

2. Recognized exhausted aliases (SSID==0, matches one of our n-N aliases,
   H-bit unset) are treated as used: we set their H-bit (correction) and
   continue scanning past them.  This is skip_complete, always on, no flag.

3. Substitution of exhausted n-N aliases is hardcoded (already done in
   9ea1197).  When the matched n-N alias has SSID==1, we replace it with our
   callsign rather than inserting + marking.

4. H-bits never unset.  Wire keeps all used addresses marked.

5. Update array is a bitmask, one byte per path location.  Bits are
   orthogonal; combinations are valid.

## Update array format

One byte per path location.  Index 0 = first digipeater address (after the
7-byte dest and 7-byte src).  Max 8 locations (MAX_PATH_ADDRS).

    bit 0  MARK_USED        set the H-bit on this address
    bit 1  DECREMENT_SSID   SSID -= 1
    bit 2  REPLACE_WITH_US  overwrite this address with our callsign (H-bit set)
    bit 3  INSERT_BEFORE    insert our callsign (H-bit set) as a NEW address
                            before this location

Bits are orthogonal.  MARK_USED | DECREMENT_SSID is valid.  REPLACE_WITH_US
implies the H-bit, so MARK_USED is redundant with it but not contradictory.
INSERT_BEFORE describes the position, not the address at that position.

## Scan rules (left to right, per location)

The scan has two phases:

Phase 1 -- find the match:
  Walk the entire path left to right. Return the first address that
  matches one of our aliases (n-N or explicit). Non-matching addresses
  do NOT block during this phase. This is a full-path search, not a
  first-unrepeated-only search.

Phase 2 -- check the blocking rule:
  Find the last used (H-bit set) address in the path. The "unused
  address index" is the position immediately after it (or 0 if no
  addresses are used). If the matched address is at a position AFTER
  the unused address index, AND the address at the unused index is
  non-empty (not a placeholder), the packet is DROPPED. If the match
  is AT the unused address index, route. If the address at the unused
  index is empty (placeholder), allow routing past it.

  Our own callsign in the path at the first unused position is not a
  blocker -- it is treated as a pass-through (the mycall-in-path check
  at lines 335-349 already handles this).

Per-location update rules (applied during rewrite, not during scan):

    H-bit already set                        -> no update, continue
    matches our n-N alias, SSID == 0         -> MARK_USED, continue
    matches our n-N alias, SSID == 1         -> REPLACE_WITH_US, stop
    matches our n-N alias, SSID > 1          -> DECREMENT_SSID | INSERT_BEFORE, stop
    matches our explicit alias               -> MARK_USED | INSERT_BEFORE, stop

INSERT_BEFORE is suppressed if the path is already at 8 addresses (no room
to grow).  The matched address is still marked/decremented/replaced.

After the scan, the rewrite executes the array location by location, then
recomputes the C-bit (address-extension bit) on the last address.  C-bit is
a frame-encoding invariant, not a routing decision: exactly one address gets
C-bit=1, all others 0.

## Routing patterns (routed cases)

### Pattern A -- n-N match, SSID > 1, room to insert
    updates[match] = DECREMENT_SSID | INSERT_BEFORE
    Cases: 108, 110, 113, 127, 133, 148, 240
    Example (108): alias WIDE1(hops=2)
        in:      N0CALL>APRS,WIDE1-2:data
        updates: [DECREMENT_SSID|INSERT_BEFORE]
        output:  N0CALL>APRS,DIGI*,WIDE1-1:data

### Pattern B -- n-N match, SSID > 1, path full (8 addrs)
    updates[match] = DECREMENT_SSID   (no insert, no room)
    Cases: 149, 233, 235, 236
    Example (233): alias WIDE2(hops=2)
        in:      N0CALL>APRS,A,B,C,D,E,F,G*,WIDE2-2:data
        updates: [0,0,0,0,0,0,0,DECREMENT_SSID]
        output:  N0CALL>APRS,A*,B*,C*,D*,E*,F*,G*,WIDE2-1:data
    Note: reference shows A,B,C,D,E,F,G*,WIDE2-1 (TNC-2 display diff only).

### Pattern C -- n-N match, SSID == 1, room to insert
    updates[match] = REPLACE_WITH_US
    Cases: 111, 117, 120, 121, 154, 169, 170, 210, 239
    Example (111): alias WIDE1(hops=1)
        in:      N0CALL>APRS,WIDE1-1:data
        updates: [REPLACE_WITH_US]
        output:  N0CALL>APRS,DIGI*:data
    DIVERGENCE: reference "no options" expected is DIGI,WIDE1* (insert +
    mark, no substitute).  We substitute because substitution is hardcoded.
    The 26 substitute_complete_n_N_address vectors in the full routes.json
    set validate this behavior.

### Pattern D -- n-N match, SSID == 1, path full
    updates[match] = REPLACE_WITH_US
    Cases: 150
    Example (150): alias WIDE3(hops=1)
        in:      N0CALL>APRS,CALL1,CALL2,CALL3,CALL4,CALL5,CALL6,CALL7*,WIDE3-1:data
        updates: [0,0,0,0,0,0,0,REPLACE_WITH_US]
        output:  N0CALL>APRS,CALL1*,CALL2*,CALL3*,CALL4*,CALL5*,CALL6*,CALL7*,DIGI*:data
    DIVERGENCE: reference shows CALL7,WIDE3*.  Same reason as Pattern C.

### Pattern E -- explicit match, room to insert
    updates[match] = MARK_USED | INSERT_BEFORE
    Cases: 41, 74, 75, 77-83, 84, 85, 87, 89, 90, 91, 94, 102, 103, 147,
           193, 195, 197, 198, 199, 201, 203, 213, 215
    Example (74): alias CALL(hops=1)
        in:      N0CALL>APRS,CALL:data
        updates: [MARK_USED|INSERT_BEFORE]
        output:  N0CALL>APRS,DIGI*,CALL*:data

### Pattern F -- explicit match, path full
    updates[match] = MARK_USED   (no insert)
    Cases: 76
    Example (76): alias DIGIA(hops=1)
        in:      N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID,DIGIE,DIGIF,DIGIG,DIGIH:data
        updates: [MARK_USED,0,0,0,0,0,0,0]
        output:  N0CALL>APRS,DIGIA*,DIGIB,DIGIC,DIGID,DIGIE,DIGIF,DIGIG,DIGIH:data

### Pattern G -- corrected exhausted alias + later match
    updates[exhausted] = MARK_USED
    updates[match]     = (Pattern A-F)
    Cases: none in the 110 "no options" set.  Exercised by skip_complete
           vectors in the explicit test suite (routes 75, 121, 134, 233, 235).
    Example: aliases WIDE1,WIDE2
        in:      N0CALL>APRS,CALLA*,WIDE1,WIDE2-2:data
        updates: [0, MARK_USED, DECREMENT_SSID|INSERT_BEFORE]
        output:  N0CALL>APRS,CALLA*,WIDE1*,DIGI*,WIDE2-1:data

## Smoke tests mapped

    rewrite_wide1_1_substitute:  Pattern C.  updates [REPLACE_WITH_US].
                                 out: N0CALL>APRS,DIGI*:hello
    rewrite_wide2_2_to_wide2_1:  Pattern A.  updates [DECREMENT_SSID|INSERT_BEFORE].
                                 out: N0CALL>APRS,DIGI*,WIDE2-1:hello

## Non-routed cases (can_repeat returns nullptr, no updates)

All routed=false cases in the 110-case array verify the scan drops correctly:

  - H-bit already set on our callsign: 57, 95, 96, 97, 98
  - Source is our callsign: 55, 63, 64, 65, 66
  - Dest is our callsign: 54
  - No alias match: 50, 67, 68, 69, 71, 72, 116, 143, 156, 202, 218, 219, 220
  - SSID == 0, no match: 70, 109
  - Unused non-matching address blocks route: 86, 105, 107, 122, 139, 144,
    145, 146, 157, 159, 167, 168, 211, 234, 241
  - Malformed packets: 223-228, 251-256

## Known divergences from reference "no options" vectors

1. H-bit display: we keep all H-bits set; reference unsets prior ones.
   Affects every case with a prior used digi.  Not a routing bug -- display
   convention.  Expected outputs here use our wire format.

2. Substitution on SSID==1: we replace with our callsign; reference (without
   the substitute flag) inserts + marks.  Affects Patterns C and D (~10
   cases).  Correct per our hardcoded-substitution decision (9ea1197).

3. Route 117 (RELAY-1*,WIDE1-1 with alias RELAY): CLOSED.  The reference
   configures TWO aliases for this case (RELAY + WIDE), but the 110-case
   array's single-alias RouteCase format can only represent one.  The scan
   with the reference's config hits RELAY-1* (H-bit set, skip) then WIDE1-1
   (matches WIDE, routes).  Not a reference quirk or prefix-matching mystery
   -- a test data extraction bug.  Resolved by test consolidation (below).

## Test consolidation (prerequisite to implementation)

The test suite is currently a mess.  Three overlapping sources of truth, two
of which assert against the WRONG behavior:

1. test_digipeater_smoke.cpp (10 tests) -- KEEP.  Hand-written, asserts our
   actual behavior (substitution hardcoded).  Correct.  No routes.json IDs.

2. test_digipeater_nn.cpp (110-case RouteCase array) -- DELETE.  Hand-extracted
   subset of routes.json with a single-alias RouteCase format that cannot
   represent multi-alias configs (cases 117, 120 broken).  Expected outputs
   assume the reference "no options" insert+mark behavior, which contradicts
   our hardcoded substitution (cases 111, 117, 120, 121, 150, 154 wrong).
   Also uses TNC-2 display convention (single '*') vs our wire format (all
   H-bits).  Every one of these is a landmine.  Retire the whole array.

3. test_digipeater_explicit.cpp -- KEEP the hand-written edge tests (the
   can_repeat_* and ssid_* TEST() blocks, ~20 tests).  DELETE the routes.json
   loader machinery (load_routes, parse_route_object, aliases_from_path,
   classify_options, run_harness) and the impl_substitute_*/impl_skip_complete_*
   tests that duplicate the conformance data with loose substring asserts.

4. test_routes_data.hpp (256 auto-generated vectors) + conformance runner --
   KEEP and PROMOTE to the single source of truth.  Wire the conformance
   runner into CMakeLists.txt as test_digipeater_conformance.  It already has
   correct multi-alias configs, options flags, and H-bit normalization.

### Substitution handling in the conformance runner

The conformance runner currently maps r.routing_mode verbatim.  Since
substitution is now HARDCODED (9ea1197), the runner must treat every n-N
SSID==1 case as substitute regardless of the vector's options bit.  Vectors
with options="" that expect insert+mark on SSID==1 (Pattern C/D) will
diverge from our output by design -- the runner must classify these as
KNOWN-DIVERGENCE (expected, not a failure) rather than FAIL.  Vectors with
options="substitute_complete_n_N_address" must PASS.  The runner prints a
divergence count separate from the failure count.

### Definition of done for consolidation

- CMake builds four targets: smoke, explicit, conformance, (nn removed).
- smoke: all pass.
- explicit: all hand-written edge tests pass; no routes.json loader remains.
- conformance: runs all 256 vectors, reports pass/fail/skip/divergence.
  Zero unexpected failures on the modes we implement (n-N routing,
  substitution, skip_complete).  Divergences are the documented Pattern C/D
  substitution cases and the H-bit display convention -- counted, not failed.
- No test asserts insert+mark as the expected output for an SSID==1 n-N match.

## Implementation outline (after review)

1. Add to DigipeaterCore member state:
       std::array<uint8_t, MAX_PATH_ADDRS> path_updates_{};
   Reset at the top of can_repeat() alongside existing match-state reset.

2. Rewrite the can_repeat() scan (DigipeaterCore.hpp:366-402) to apply the
   scan rules above and populate path_updates_.  Case "unused, matches
   nothing" returns nullptr (the blocking fix).

3. Make the fallback scan inside rewrite_frame_impl() populate the same
   path_updates_ so both paths agree (single source of truth).

4. Replace the address-removing skip_complete block
   (rewrite_frame_impl:512-537) with: for each location, apply MARK_USED /
   DECREMENT_SSID / REPLACE_WITH_US; if INSERT_BEFORE, shift and insert our
   callsign.  Keep all addresses in the path (nothing removed).

5. Recompute C-bit on the last address (already done at :581-587).

## Verification plan

Baseline before change: smoke 9/9, n-N 71/110, explicit 22/8/7.

After change: run all three suites.  Expectation:
  - The "unused non-matching address blocks route" cases should move from
    whatever they do now to correctly dropping.
  - Pattern G cases in the explicit suite (the 8 failures) should improve,
    since skip_complete now keeps addresses and sets H-bits instead of
    removing them.
  - Any case that currently passes and starts failing is a real signal; stop
    and report before proceeding.

Report the before/after delta explicitly.
