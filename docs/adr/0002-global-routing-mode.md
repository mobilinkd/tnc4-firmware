# 0002: Global routing mode — not per-alias preempt or tracing

The digipeater alias struct originally had per-alias `preempt` (bool) and `insert_id` (bool) fields. We removed both and consolidated into a single global `routing_mode` byte in the digipeater settings struct.

A future reader would expect preempt behavior to be per-alias — "this WIDE1 alias should preempt-front, but this TRACE alias should just mark." That's the intuitive design. The reason we didn't do it: preempt behavior is a routing policy, not a per-alias property. It applies to the digipeater's treatment of *all* paths, not how it matches individual aliases. A digipeater doesn't switch preempt modes based on which alias triggered the match — it has one consistent strategy for path modification.

Per-alias preempt would also complicate the config UI (iOS/Android apps) and the libaprsroute integration (which treats preempt as a global routing option).

## Considered Options

**Per-alias preempt/insert_id fields.** Rejected — preempt is a routing policy, not an alias property. Per-alias fields would let users configure contradictory modes (e.g. preempt-front on WIDE1, preempt-drop on WIDE2), which libaprsroute can't express. Also reduces struct size by 2 bytes per alias (16 bytes saved across 8 aliases).

**Global routing mode byte.** Selected — one byte with bit flags for preempt mode, substitution, and traceless options. Clean separation: aliases define *what* to match, routing mode defines *how* to modify the path.
