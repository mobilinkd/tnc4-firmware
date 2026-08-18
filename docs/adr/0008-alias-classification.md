# ADR-0008: Alias classification — class-mask bitfield

**Status:** Accepted
**Date:** 2026-08-18

## Context

The digipeater alias model has been through two candidate designs, both
revolving around how the firmware decides whether a configured alias is an
*n-N* alias (a routing prefix like `WIDE`, `TRACE`) or an *explicit* alias
(a full callsign like `RELAY` or a specific digi's call).

The first model auto-classified by prefix: `is_nN_alias()` hardcoded the
strings `WIDE`, `TRACE`, `RELAY`, `ECHO`, `GATE`, `TEMP` as n-N and
treated everything else as explicit.  This is wrong on two counts:

- The APRS spec (aprsspec) notes that `RELAY`, `TRACE`, and plain `WIDE`
  have been obsolete since ~2004, so a prefix list is not stable ground.
- No reference implementation auto-classifies by prefix.  libaprsroute
  separates `n_N_addresses` from `explicit_addresses`; Direwolf separates
  `wide_pattern` from `alias_pattern`.  In both, the *operator* marks the
  classification.

The second candidate (recorded in the session notes as "decision 8")
marked classification via the existing `hops` field: `hops == 0` meant
explicit, `hops > 0` meant n-N with `hops == class n`.  It also locked
"decision 7", a firm requirement that `N > n` is never digipeated,
encoded by `ssid <= a.hops`.

Both *classification* candidates are replaced by this decision.  The
`hops` field is removed outright; classification and the set of served
classes are encoded in a single bitfield.  Decision 7 (`N > n` is never
digipeated) is **retained** and re-encoded as `ssid <= class digit`
(Decision 5) rather than `ssid <= a.hops`.

## Decision 1: the `set` field becomes a class-mask bitfield

The `Alias` struct's `set`, `use`, and `hops` fields are replaced by a
single `uint8_t flags` field:

- **bit 0** — `use`: the alias is enabled for digipeating.
- **bits 1-7** — the *class mask*: bit *n* (n = 1..7) selects n-N class
  *n*.  The operator sets exactly the classes this alias serves.

```c++
struct Alias {
    call_t call;      // routing prefix (n-N) or full callsign (explicit)
    uint8_t flags;    // bit 0 = use; bits 1-7 = class mask
};
```

The `Alias` struct is now 9 bytes (was 11).  The `Hardware` struct and
the stored EEPROM image shrink by 16 bytes (8 aliases x 2 bytes).

## Decision 2: "set" is derived from the callsign

An alias slot is configured when its callsign is non-empty.  The meaning
previously carried by the `set` flag is now `call.callsign[0] != '\0' &&
call.callsign[0] != ' '` (NUL and space are both "empty" so that a
space-padded zeroed slot is not mistaken for a configured alias).

## Decision 3: operator-marked classification via the mask

- **n-N alias**: the class mask (bits 1-7) is non-zero.  The callsign is
  the routing *prefix* (e.g. `WIDE`), and matching is prefix + class
  digit + mask (Decision 4).
- **explicit alias**: the class mask is zero (only `use` set).  The
  callsign is matched by full-callsign equality.

Any address with at least one class bit set is an n-N address.  A user
digipeats exactly the collection of classes whose bits are set, e.g.
`WIDE` with bits 1 and 2 serves `WIDE1-*` and `WIDE2-*` and nothing else.

## Decision 4: n-N matching semantics

A 7-byte path address matches an n-N alias when:

1. its callsign starts with the alias prefix (non-space length L, 1..5),
2. the character at position L is a class digit `1`..`7`,
3. that class's bit is set in the alias's class mask,
4. the remaining callsign positions are spaces, and
5. the SSID (the address's `N`) is in the range 1..n, where n is the
   matched class digit (Decision 5).

Explicit aliases match by full 6-character callsign equality (SSID
insensitive, preserving the pre-existing explicit-match behavior).

## Decision 5: the "N > n" firm requirement is retained

Decision 7 holds unchanged: a packet whose n-N address has a remaining-hop
count `N` greater than its class digit `n` (e.g. `WIDE2-3`) is abuse or
corruption and is never digipeated.  Under the mask model this is encoded
as `ssid <= class digit`: the mask selects *which* classes are served, and
for a matched class `n` the address's `N` must be in `1..n`.

The reference (libaprsroute) serves any `N` in 1..7 on a configured class;
Direwolf's regex model likewise treats `n` and `N` as independent axes.
Retaining the `N <= n` cap is a deliberate, spec-driven divergence from
both references: the APRS spec's n-N paradigm defines `n` as the class and
`N` as the remaining hop count, so `N > n` is a nonsensical path.

## Wire format

`EXT_GET_ALIAS` / `EXT_SET_ALIAS` and the `EXT_GET_ALL_DIGIPEATER_CONFIGS`
bulk reply change from 12 bytes per alias to 10 bytes per alias:

```
Old: [index(1)][call(8)][set(1)][use(1)][hops(1)]
New: [index(1)][call(8)][flags(1)]
```

The digipeater feature has never shipped, so this is not a compatibility
break against any released firmware.  The config app (tnc1-python-config)
must be updated to the new 10-byte alias layout and to build `flags` from
its alias UI.

## Considered options

**Keep `hops` as a redundant field alongside the mask.**  Rejected.
`hops` would carry no information the mask does not already encode, and
its prior overload (class and max-N) is exactly the confusion this ADR
removes.

**Keep the 11-byte struct with two reserved bytes.**  Rejected.  The
digipeater config is unreleased; the reserved bytes would be dead weight
and would re-open the question of their meaning later.

**Auto-classify by prefix (`is_nN_alias`).**  Rejected.  No reference
does this; classification is the operator's to mark.

## Consequences

- The `N > n` tests are retained, adapted to the class-mask alias
  construction: a served class routes `N` in `1..n` and drops `N > n`.
- The conformance runner classifies the six reference-routes / we-reject
  `N > n` vectors (108, 110, 113, 127, 133, 134) as policy divergences.
- The conformance runner maps each route's n-N alias to a class mask
  (the generated data encodes the class digit, or `0` for a generic
  prefix that serves every class 1..7).
- `is_nN_alias()` and the `a.hops` field are removed from the firmware.
- The `Hardware` EEPROM image shrinks by 16 bytes; a stored image from
  this unreleased branch fails CRC on load and resets to defaults, which
  is accepted.
