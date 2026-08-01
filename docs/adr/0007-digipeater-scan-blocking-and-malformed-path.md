# ADR-0007: Digipeater scan — blocking rule and malformed-path policy

**Status:** Accepted
**Date:** 2026-07-31

## Context

The digipeater scan decides whether a received packet matches one of our
aliases and should be routed.  Two behaviors shape that decision:

1. A packet may carry non-empty, unused (H-bit clear) addresses ahead of
   the alias we would route on.  libaprsroute refuses to route such
   packets: a stray unused callsign means the packet already traveled
   somewhere unexpected, and routing it creates loops and duplicates.

2. A packet may carry an empty (placeholder) path slot: a 7-byte address
   whose callsign field is all spaces.  libaprsroute routes past an empty
   first-unused slot and removes the slot during rewrite.

## Decision 1: blocking rule (2026-07-31, commit 0722729)

Route only on the first unused address after the used prefix.  The scan
is two-phase:

- Phase 1: walk the entire path left to right and find the first address
  matching one of our aliases (n-N or explicit).  Non-matching addresses
  do not block during this phase.
- Phase 2: if the match is at a position AFTER the first unused position
  (the address immediately after the last H-bit-set address, or the
  first path address if none are used), and the address at the first
  unused position is a real address that is not our own callsign, drop
  the packet.

Our own callsign at the first unused position is a pass-through (the
mycall-in-path check handles loops separately).

Rationale: matches the libaprsroute reference (try_n_N_route /
try_explicit_route).  A stray unused callsign ahead of the match means
the packet has already traveled somewhere unexpected; routing it would
create loops and duplicate packets.

## Decision 2: malformed-path policy (2026-07-31)

If the first unused path slot is empty (all-space placeholder) and there
is any non-empty unused address after it, the packet is malformed: the
path has a hole.  Malformed packets are dropped.

- We do not repeat malformed packets.
- We do not clean up malformed packets (no trimming of empty slots
  during rewrite).
- We drop them.

This is a deliberate divergence from libaprsroute, which routes past an
empty first-unused slot and removes the slot during rewrite ("we should
still allow empty addresses to be routed so they can be removed").
Conformance case 221 (N0CALL>APRS,,WIDE1-1:data) exercises this: the
reference routes it and trims the hole; we drop it as malformed.  The
conformance runner classifies 221 as a known divergence, not a failure.

Rationale: an empty slot followed by content is a broken digipeater
path.  The reference's route-and-trim behavior repairs malformed input;
we do not repair malformed input.

## Implementation

- `DigipeaterCore.hpp`: `first_unused_offset()`,
  `is_empty_path_address()`, and `blocked_by_unused_address()` encode
  both decisions.  Used by both the primary scan (`can_repeat()`) and
  the fallback scan in `rewrite_frame_impl()`.
- Conformance runner: routing-decision mismatches where the input's
  first unused slot is empty are classified as a policy divergence
  (case 221), consistent with the Pattern C/D substitution divergences.

## Considered options

**Allow routing past an empty slot, reference-style.** Rejected.  The
reference's empty-slot allowance exists so the rewrite can remove the
hole — i.e. cleaning up malformed packets.  We do not clean up malformed
packets.

**Treat empty slots as ordinary non-matching addresses.** Equivalent in
effect to Decision 2 (they never match an alias and never satisfy the
mycall exemption), but leaves the policy implicit.  Rejected: the
malformed-path policy is explicit so the drop is intentional and
documented, not an accident of callsign comparison.

## Consequences

- Packets with holes in their digipeater path are never repeated.
- The conformance suite reports 92 passed / 9 diverged / 41 failed /
  114 skipped; 221 is in the divergence bucket.
- Trackers and digis that pre-allocate fixed-width paths and blank
  unused slots will see their packets dropped if the blank slot precedes
  content.  This is accepted: such packets are malformed.
