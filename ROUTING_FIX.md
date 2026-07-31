# ROUTING_FIX.md -- Preempt Routing Conformance

Status: DRAFT -- pending Rob's review
Date: 2026-07-30
Branch: feature/digipeater-integration

## Problem

Our digipeater routing does not conform to the APRS spec.  The reference
implementation is Direwolf; libaprsroute (forked at
/home/hermes/.hermes/profiles/kyle/workspace/libaprsroute) encodes the same
semantics with 256 concrete test vectors in tests/routes.json.

We are NOT free to define our own semantics.  The semantics are defined by the
APRS spec and Direwolf is the reference.

## Current State

### What works (conforms)

- n-N routing: WIDE/TRACE/RELAY/ECHO/GATE/TEMP prefix aliases, SSID hop
  decrement, H-bit marking.  71/110 test vectors pass; 46 documented diffs
  from libaprsroute (mostly insertion-policy differences, see ADR-0006).
- Substitution of exhausted n-N aliases: hardcoded (always on).  See
  KissTypes.hpp.  Not configurable.
- ROUTING_SKIP_COMPLETE (0x80): drop completed (ssid==0, H-bit set) addresses
  before the match.  Implemented and tested.

### What is broken

**Bug 1: ROUTING_PREEMPT_FRONT (0x01) -- REMOVED (5d9999f).**

Preempt routing was removed entirely.  See ADR-0002 and the preempt
provenance section below.  The bits 0x01-0x08 are reserved and ignored.

## Preempt mode provenance (added 2026-07-31)

The four preempt modes in libaprsroute do NOT all come from the same source.
Only two are in the APRS spec.  One is a Direwolf enhancement.  One is a
libaprsroute invention based on an unimplemented idea from a code comment.

Sources:
- APRS spec: WB4APR "Preemptive Digipeating" (aprs.org/aprs12/preemptive-digipeating.txt)
  Defines three settings: OFF, DROP, MARK.  No other preempt modes.
- Direwolf (wb2osz/direwolf, src/digipeater.c): implements PREEMPT_OFF,
  PREEMPT_DROP, PREEMPT_MARK, and PREEMPT_TRACE.  TRACE is WB2OSZ's own
  enhancement ("My enhancement - remove prior unused digis") to provide an
  accurate path trace.  Not in the APRS spec.
- libaprsroute (iontodirel/libaprsroute, aprsroute.hpp:270-437): defines
  preempt_front, preempt_truncate, preempt_drop, preempt_mark.
  preempt_truncate == Direwolf's PREEMPT_TRACE (same semantics: erase
  addresses between last-used and our position, re-insert us there).
  preempt_front is NOT in Direwolf and NOT in the APRS spec.  It corresponds
  to an unimplemented idea in a code comment in digipeater.c:

      // Idea: Here is an interesting idea for a new option. REORDER?
      // The preemptive digipeater could move its call after the (formerly)
      // last used digi field and preserve all the unused fields after that.

  No digipeater in the field produces or consumes preempt_front paths.

Mapping:

| libaprsroute     | Direwolf        | APRS spec | Provenance               |
|------------------|-----------------|-----------|--------------------------|
| preempt_front    | (not implemented)| no        | libaprsroute invention   |
| preempt_truncate | PREEMPT_TRACE   | no        | Direwolf enhancement     |
| preempt_drop     | PREEMPT_DROP     | DROP      | APRS spec                |
| preempt_mark     | PREEMPT_MARK     | MARK      | APRS spec                |

Decision: Initially support DROP and MARK only (the two APRS spec modes).
TRACE and FRONT are deferred pending further research.  The routing_mode
bit assignments in KissTypes.hpp will need to be revised to reflect this --
the current 0x01 (PREEMPT_FRONT) bit is mislabeled and will be reassigned.

**Bug 2: ROUTING_SKIP_COMPLETE (0x80) is wrong in two ways.**

libaprsroute definition (aprsroute.hpp:333-341):

    Skip complete n-N addresses even if unset.
    The completed/unset address has to be in the n_N_addresses list.

    This packet: N0CALL>APRS,CALLA*,WIDE1,WIDE2-2:data
                                      ~~~~~
    Will be routed as: N0CALL>APRS,CALLA,WIDE1,DIGI*,WIDE2-1:data

Purpose: during MATCHING, skip past exhausted n-N aliases (SSID=0) so the
digi routes on the next live alias instead of trying to match on a dead one.
The key phrase is "even if unset" -- the exhausted alias may NOT have the
H-bit, because a broken upstream digi decremented WIDE1-1 to WIDE1-0 without
marking it used or removing it.

Our implementation (DigipeaterCore.hpp:552-558):

    if (skip_complete && i < match_idx) {
        uint8_t ssid = (ssid_byte >> 1) & 0x0F;
        bool h_bit = (ssid_byte & 0x80) != 0;
        if (ssid == 0 && h_bit) continue;   // skip this address
    }

Two bugs:

(a) Wrong phase.  We apply skip_complete in the REWRITE (removing addresses
    from the output path).  libaprsroute applies it during MATCHING (skipping
    past exhausted aliases when finding what to route on).  Note the example
    output: WIDE1 is STILL IN THE PATH.  It wasn't removed.  The digi just
    skipped past it for matching and routed on WIDE2-2 instead.

(b) Wrong condition.  We check `ssid == 0 && h_bit`.  But the whole point is
    "even if unset" -- the H-bit is NOT set on the exhausted alias.  That's
    the broken-digi scenario we're defending against.  Our condition requires
    the H-bit, which means we only skip addresses that are already properly
    marked as used -- the case that doesn't need help.  The correct condition
    is: address matches a configured n-N prefix AND ssid == 0, regardless of
    H-bit.

Additionally, our filter applies to ALL addresses before the match, not just
n-N aliases.  An explicit callsign like WB2OSZ* (SSID=0, H-bit set) would be
incorrectly skipped.  libaprsroute requires "the completed/unset address has
to be in the n_N_addresses list."

## Reference Algorithm (libaprsroute aprsroute.hpp:1617-1931)

The reference uses a three-stage pipeline:

### Stage 1: Match

Find `maybe_router_address_index` (where our callsign or an explicit alias
appears in the path) and `unused_address_index` (first address without H-bit).
This decides whether to route and where we are in the path.

### Stage 2: Transform (try_preempt_transform_explicit_route)

Restructure the path per mode:

- preempt_front: `try_move_address_to_position(from=our_pos, to=first_unused)`
  -- removes our address and re-inserts it at the first-unused slot.
- preempt_truncate: `try_truncate_address_range(start=first_unused, end=our_pos)`
  -- erases the range and re-inserts our address at the start.
- preempt_drop: `try_truncate_address_range(start=0, end=our_pos)` then reset
  unused_index=0 -- erases everything before us.
- preempt_mark: no path change, just set unused_index = our_pos.

### Stage 3: Basic Route (try_explicit_basic_route)

- Unmark H-bits on addresses ahead of us (they were marked by previous digis
  but we're preempting past them).
- Mark our address with H-bit.
- For path-based matches (alias matched, not our literal callsign): insert our
  callsign before the matched alias, or replace the alias if path is full (8
  addresses).

### Key detail: H-bit semantics (CORRECTED)

The APRS spec (APRS-Digipeater-Algorithm.pdf, WB2OSZ) is unambiguous:

> "Behind the scenes, in the AX.25 frame, each digipeater address has a
> 'has been used' H bit to indicate that the address has already been used."

The H-bit is set when a digi routes the packet and is NEVER unset.  Real-world
capture data confirms this: in 55 multi-digi frames from /tmp/cutecom.log,
every digi that routed the packet has its H-bit set.  Zero exceptions.

The TNC-2 monitoring DISPLAY format shows `*` only after the LAST used
digi -- earlier used addresses are implied.  The spec explicitly says:

> "Some software might display it like this with two '*' characters.
>  WB2OSZ>APZ,N2GH*,W2UB*:something   Wrong"

This is a display convention, not a wire-format rule.  On the wire, ALL
used addresses carry the H-bit.

libaprsroute's `set_address_as_used()` calls `unset_all_used_addresses()`
before marking the current address.  This means its test vectors show only
one `*` in the output string.  This is the TNC-2 display convention baked
into the library's internal state.  When building the conformance harness,
we must compare at the byte level (actual H-bits), not at the display
string level, because our wire output will have all H-bits set while
libaprsroute's test vector strings show only the last `*`.

Our `ax25_packet_to_string()` in the test harness deliberately shows `*`
on ALL H-bit-set addresses, not just the last one.  This deviates from the
TNC-2 display convention but is more useful for debugging and routing
verification.  This is documented in the code.

Our firmware correctly never clears H-bits.  This is NOT a conformance gap.

## Structural Mismatch: Alias Model

libaprsroute separates `explicit_addresses` (aliases triggering explicit/
preempt routing) from `n_N_addresses` (WIDE/TRACE hop-count aliases).  Our
firmware has a single `aliases[]` array and auto-classifies by prefix
(`is_nN_alias`: WIDE/TRACE/RELAY/ECHO/GATE/TEMP -> n-N, else explicit).

For preempt purposes this is fine -- preempt operates on the explicit side.
But it's where subtle divergence can hide, especially for combined-mode
vectors (e.g. preempt_front + substitute_explicit_address).

## Proposed Plan

### Phase 0: Conformance harness (no firmware changes)

Build a test that loads all 256 routes.json vectors, maps each route's
`options` string to our routing_mode bitmask, runs each through
DigipeaterCore::can_repeat + rewrite_frame, and diffs against the reference
`routed_packet`.  Produces a precise failure baseline: exactly which vectors
we fail, with input/expected/actual for each.

This converts "I think preempt_truncate does X" into "we fail these N vectors,
here's the exact diff."  Every subsequent change is measured against ground
truth.

Mapping from libaprsroute options to our flags:

| libaprsroute option | Our flag | Notes |
|---------------------|----------|-------|
| preempt_front | ROUTING_PREEMPT_FRONT (0x01) | |
| preempt_truncate | ROUTING_PREEMPT_TRUNCATE (0x02) | |
| preempt_drop | ROUTING_PREEMPT_DROP (0x04) | |
| preempt_mark | ROUTING_PREEMPT_MARK (0x08) | |
| substitute_complete_n_N_address | (hardcoded) | Always on, no flag |
| skip_complete_n_N_address | ROUTING_SKIP_COMPLETE (0x80) | |
| substitute_explicit_address | (no flag yet) | Needs a new bit |
| traceless_n_N_route | (no flag yet) | Needs a new bit |
| trap_limit_exceeding_n_N_address | -- | REJECTED -- see below |
| reject_limit_exceeding_n_N_address | -- | REJECTED -- see below |
| route_self | (no flag yet) | Deliberate QRM risk; hidden/debug only |
| strict | (no flag yet) | Validation only, not routing |
| preempt_n_N | (no flag yet) | Preempt on n-N packets |

Our routing_mode byte has bits 0x10 and 0x20 free.  That's only 2 bits; the
reference has 13 options.  We may need to decide which options to expose and
which to hardcode.

### TRAP and REJECT: explicitly rejected (2026-07-31)

libaprsroute defines trap_limit_exceeding_n_N_address (replace abusive alias
with our callsign, kill the path) and reject_limit_exceeding_n_N_address
(drop the packet entirely).  These exist to police the network against
WIDE7-7 flooding.

Decision: neither is implemented, and neither will be.  The TNC4 is a
personal/mobile digi with an explicit alias list (no regex).  If an address
doesn't match a configured alias, it is ignored -- the packet passes through
untouched.  This is already the conservative behavior.  Actively trapping
adds airtime cost, callsign-substitution complexity, and config surface for
a scenario that matters to high-traffic fixed digis, not personal devices.

Consequence: the Alias.hops field exists solely to distinguish "normal n-N
that decrements" from "trap that callsign-substitutes."  With no trap, hops
is vestigial.  The `ssid <= a.hops` check in DigipeaterCore.hpp currently
acts as an implicit N>n filter (e.g. WIDE2 with hops=2 silently ignores
WIDE2-3).  Removing hops would change that behavior -- WIDE2-3 would match
and be decremented.  This is a separate design decision, tracked below.

Open question: deprecate Alias.hops?  Direwolf's wide pattern matches
^WIDE[1-7]-[1-7]$ with no hop-count filter.  If we remove hops, we match
Direwolf's behavior but lose the implicit N>n guard.  If we keep hops, we
diverge from Direwolf but retain a simple sanity filter.  Either way, the
field's original purpose (trap signaling) is gone.

### Phase 1: Fix preempt_front

Restructure rewrite_frame_impl to implement the match/transform/basic-route
pipeline.  Fix preempt_front to actually move our address to the first-unused
position.  Add H-bit unmarking for addresses ahead of us.

### Phase 2: Implement preempt_truncate, preempt_drop, preempt_mark

Each is a distinct transform in the pipeline.  All four preempt modes share
the same match and basic-route stages.

### Phase 3: Combined modes + alias classification

Handle vectors with multiple options set (e.g. preempt_front +
substitute_explicit_address).  Reconcile explicit/n-N alias classification
with the reference.

### Phase 4: Documentation

Correct config-app-requirements.md routing flag table.  Write an ADR
documenting the conformance fix and the alias-model simplification.

## Open Questions

1. Alias model: keep single-array + prefix auto-classification, or add a
   per-alias explicit/n-N flag to match libaprsroute exactly?

2. routing_mode byte is 8 bits; the reference has 13 options.  Which options
   do we expose as config bits vs hardcode?  SUBSTITUTE is hardcoded (always
   on, matches Direwolf).  SKIP_COMPLETE is configurable.  The four preempt
   modes are reserved and ignored.  Bits 0x10 and 0x20 are free for
   substitute_explicit and traceless if needed.

3. route_self: deliberate QRM risk.  Recommend hidden/debug-only, not a
   config-app toggle.

## Why the tests don't catch these bugs

This is the most important section.  We have 100+ tests and they pass.  Two
of the three implemented routing modes are wrong.  How?

### Root cause: tests were written against our behavior, not the spec

The tests assert what the code DOES, not what the spec SAYS.  When the code
is wrong, the tests codify the wrong behavior and pass.  This is the same
false-confidence trap as the Python call_t tests -- assertions written to
match the implementation, not the contract.

### Bug 1 (PREEMPT_FRONT): test vectors were hand-written to match our output

The preempt_front tests (test_digipeater_explicit.cpp, routes 77-83, 147)
were written by a subagent that ran our code, observed the output, and wrote
assertions matching that output.  The expected strings in the RouteVec
structs describe mark-in-place behavior, not move-to-front.  The tests pass
because they assert the bug.

libaprsroute's routes.json has 13 preempt_front vectors with the correct
move-to-front expected output.  We never ran against those.

### Bug 2 (SKIP_COMPLETE): three compounding gaps

(a) The n-N test suite (test_digipeater_nn.cpp, 110 cases) runs with
    routing_mode=0 -- no SKIP_COMPLETE flag set.  The skip_complete code
    path is never exercised by the n-N tests.

(b) The explicit test suite has 4 skip_complete tests (routes 75, 121, 134,
    233).  These use weak assertions:

        EXPECT_TRUE(out.find("DIGI") != std::string::npos);

    This checks "DIGI appears somewhere in the output."  It does NOT check:
    - Whether WIDE1 (the exhausted alias) is still in the path (spec: yes)
    - Whether WIDE1 was removed from the path (our bug: yes, wrongly)
    - Whether the match happened on the correct alias
    - The exact output packet

    A substring find() for "DIGI" passes regardless of whether skip_complete
    removed addresses, kept them, or did nothing at all.

(c) The critical test vector -- the "even if unset" case -- is missing.
    libaprsroute route #160:

        in:  N0CALL>APRS,WIDE1,WIDE2-2:data
        out: N0CALL>APRS,WIDE1,DIGI*,WIDE2-1:data

    WIDE1 has SSID=0 and NO H-bit.  A broken upstream digi left it there.
    skip_complete should skip past it during matching and route on WIDE2-2.
    WIDE1 stays in the output path.

    Our code would never produce this output because:
    - Our condition requires h_bit=true, but WIDE1 has no H-bit
    - Our code removes addresses instead of skipping during matching
    - We'd either match on WIDE1 (wrong) or not match at all

    This vector is not in our test suite.  The 4 vectors we DO test all have
    properly-marked addresses (H-bit set), which is the easy case.

### Structural problem: no conformance baseline

We have no test that runs all 256 routes.json vectors through our code and
reports pass/fail against the reference expected output.  Without that, each
test is an isolated assertion that can drift from the spec independently.
The Phase 0 conformance harness fixes this structurally.

### The pattern

    1. Code is written from a prose understanding of the spec.
    2. Tests are written to match the code's observed behavior.
    3. Tests pass.  Confidence is high.  Behavior is wrong.

The fix is not "write more tests."  It's "write tests against the reference
vectors, not against our own output."  routes.json has 256 vectors with
exact expected output.  That's the test suite.  We just need to run it.

## Test Vector Summary (routes.json)

| Category | Count | Our status |
|----------|-------|------------|
| n-N routing (no options) | 110 | 71 pass, 39 fail (insertion-policy diffs) |
| substitute_complete_n_N_address | 26 | Hardcoded (always on) |
| substitute_explicit_address | 21 | Not implemented |
| preempt_truncate | 18 | Not implemented |
| traceless_n_N_route | 14 | Not implemented |
| preempt_front | 13 | WRONG behavior |
| preempt_drop | 8 | Not implemented |
| trap_limit_exceeding | 4 | REJECTED -- not implementing |
| reject_limit_exceeding | 4 | REJECTED -- not implementing |
| skip_complete_n_N_address | 4 | Implemented |
| preempt_mark | 1 | Not implemented (but is what FRONT does) |
| route_self | 2 | Not implemented |
| strict | 1 | Not implemented |
| Combined modes | ~20 | Not implemented |
| **Total** | **256** | |
