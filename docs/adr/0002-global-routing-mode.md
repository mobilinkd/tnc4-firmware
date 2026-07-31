# 0002: Global routing mode — no preemptive digipeating

The digipeater alias struct originally had per-alias `preempt` (bool) and `insert_id` (bool) fields. We removed both and consolidated into a single global `routing_mode` byte in the digipeater settings struct.

## Decision: no preemptive digipeating (2026-07-31)

Preemptive digipeating is explicitly unsupported.  The routing_mode bits
0x01-0x08 (ROUTING_PREEMPT_FRONT, ROUTING_PREEMPT_TRUNCATE,
ROUTING_PREEMPT_DROP, ROUTING_PREEMPT_MARK) are reserved and ignored by the
routing engine.  The constants are retained in KissTypes.hpp for documentation.

Rationale: there is no formal standard for preemptive digipeating.

- The APRS spec (WB4APR, preemptive-digipeating.txt) defines DROP and MARK,
  but a second spec document (RR-bits.txt) disagrees on RR-bit handling.
  The two documents were never reconciled.
- No implementation follows the spec exactly.  Direwolf ignores RR bits,
  implements DROP/MARK/TRACE, and deprecates DROP and MARK in favor of TRACE
  (which is not in the spec at all).  libaprsroute adds a fourth mode
  (preempt_front) based on an unimplemented idea from a code comment in
  Direwolf's source.
- Community discussion (TAPR aprssig, 2006-2016) found no consensus on
  preemptive digipeating.  One contributor called it "ridiculous" and
  "unreliable" due to lack of standard implementation and legacy hardware
  that won't change.
- The feature is sparsely used in practice.  Real-world capture data
  (/tmp/cutecom.log, 138 AX.25 data frames) shows zero preempt routing.

The routing_mode byte retains SKIP_COMPLETE (0x80) as the only configurable
flag.  Substitution of exhausted n-N aliases is hardcoded -- Direwolf does
this unconditionally and no implementation in the field runs without it.

## Original rationale for global (not per-alias) routing mode

A future reader would expect preempt behavior to be per-alias — "this WIDE1 alias should preempt-front, but this TRACE alias should just mark." That's the intuitive design. The reason we didn't do it: preempt behavior is a routing policy, not a per-alias property. It applies to the digipeater's treatment of *all* paths, not how it matches individual aliases. A digipeater doesn't switch preempt modes based on which alias triggered the match — it has one consistent strategy for path modification.

Per-alias preempt would also complicate the config UI (iOS/Android apps) and the libaprsroute integration (which treats preempt as a global routing option).

## Considered Options

**Per-alias preempt/insert_id fields.** Rejected — preempt is a routing policy, not an alias property. Per-alias fields would let users configure contradictory modes (e.g. preempt-front on WIDE1, preempt-drop on WIDE2), which libaprsroute can't express. Also reduces struct size by 2 bytes per alias (16 bytes saved across 8 aliases).

**Global routing mode byte.** Selected — one byte with bit flags for preempt mode, substitution, and traceless options. Clean separation: aliases define *what* to match, routing mode defines *how* to modify the path. Preempt bits are now reserved and ignored.
