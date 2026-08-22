# Classifier Injection + Routing-Gap Closure Plan

Date: 2026-08-18
Branch: feature/digipeater-integration
Status: PLAN -- pending Rob's review. No code changed.
Supersedes (as analysis source): HANDOFF.md "Remaining firmware gaps" section.
Complements: docs/plans/2026-07-31-routing-update-array.md (update-array design).

## TL;DR

Two problems, one prerequisite:

1. The conformance test data cannot express the operator-marked alias model
   (ADR-0008).  `tests/gen_conformance_data.py` re-derives alias
   classification from string *syntax* (parse_nn) instead of carrying the
   reference's per-list classification as data.  No alias is special in the
   firmware -- the *generator* is what makes RELAY always-explicit and WIDE
   always-n-N.  This must be fixed before the E/F routing work is testable.

2. The 36 conformance failures reveal five real routing-rule deficiencies,
   NOT one monolithic "explicit routing not implemented".  Re-bucketed with
   reference-verified semantics (below).  The largest bucket is
   **self-routing** (mycall as the routed address), which the handoff did
   not call out.

Plus: two failures (169, 170) and one interaction case (231) are TEST
HARNESS limitations, not firmware bugs.  Handled at the harness level.

## 1. The classifier problem -- "no alias is special"

### Current state

Firmware (ADR-0008, KissTypes.hpp): `Alias { call_t call; uint8_t flags }`
with bits 1-7 = class mask.  Non-zero mask -> n-N; zero -> explicit.
`alias_call_matches()` branches only on `is_nN()`.  The type system is
already uniform.

Test data: routes.json carries three lists per route: `path`
(auto-classified by the reference), `explicit_addresses` (caller-marked
explicit), `n_N_addresses` (caller-marked n-N).  `gen_conformance_data.py`
collapses all three into a single `classes` byte using **string syntax**
(`parse_nn`) as the sole classifier.  Consequences:

- `"RELAY"` can never be n-N (no digit) -- RELAY is special by accident.
- `"RELAY2"` is always n-N (syntactically `PREFIXn`).
- `"WIDE1-3"` in explicit_addresses is always explicit (caller marking
  honored for explicit, re-derived for n-N -- an asymmetry).
- No vector can express "operator marked RELAY as n-N" or "operator
  marked plain WIDE as n-N".
- No vector carries a per-alias classification at all -- it is computed
  at generate time and frozen into test_routes_data.hpp.

### Fix: carry classification as data

1. Extend the generated data: each alias entry gains a classification
   field, e.g. `enum class AliasClass : uint8_t { Explicit = 0, NN = 1 }`
   with `struct AliasEntry { const char* call; uint8_t classes; uint8_t cls; }`.
   The conformance runner maps cls -> make_alias() unchanged; the `classes`
   byte is still the class mask (bits 1-7), zero for explicit.

2. The generator derives classification from **the reference's lists**,
   not from syntax:

   - `explicit_addresses` -> explicit, always.
   - `n_N_addresses` -> n-N (validation: parse_nn must succeed; if it
     fails, the entry is unrepresentable -- warn and drop the vector).
   - `path` -> keep the reference's auto-classification (it is part of the
     reference's semantics: a `path` entry that parses as `PREFIXn` is
     n-N, else explicit).  This is a property of the reference, not our
     model; it stays syntactic ONLY for the `path` list.

3. Add new explicit-classification vectors where the syntax-based model
   and the operator model differ, so the firmware's model is actually
   exercised:
   - explicit "WIDE" (plain) served as an explicit alias with classes=0
     (already possible; needs at least one positive PASS vector).
   - "RELAY" with class mask (n-N) -- only in a hand-written vector in
     test_digipeater_explicit.cpp via make_alias("RELAY", 0x04); routes.json
     cannot express it and must not.  NOTE: class 2 = bit 2 = 0x04 (ADR-0008
     class-mask encoding), NOT 0x02 (that is class 1).
   - "WIDE2" with mask=0 (explicit, not n-N) -- same, hand-written only.
4. Regenerate test_routes_data.hpp and re-run; the runner must report
   zero change for existing vectors (classification of the current data
   set is syntax-consistent by construction).

This is a data-path change only. No firmware impact.  Definition of done:
`python3 tests/gen_conformance_data.py > tests/test_routes_data.hpp` +
full suite; conformance = 93 passed / 14 diverged / 33 failed / 116 skipped
(231 flips FAIL->PASS via harness SSID clamp; 169/170 flip FAIL->SKIP via
generator unrepresentable marking; 2 new hand-written explicit vectors raise
the explicit suite to 26 passed).  The "92/14/36/114" unchanged target in the
section-1 DoD is superseded by this corrected delta.

## 1.5 Firmware requirement: set-alias must reject unrepresentable n-N aliases

New requirement (2026-08-18): the KISS `EXT_SET_ALIAS` command must zero the
config byte (the `flags` byte: bit 0 = use, bits 1-7 = class mask) when it
sets an n-bit on a 6-character callsign.  A 6-character prefix leaves no room
for the n-N class digit (`PREFIXn` needs 7 wire characters), so such an alias
is unrepresentable -- the same condition that makes conformance cases 169/170
SKIP at the generator level.  The matcher already refuses to match such a
prefix (ADR-0008, `plen >= 6`), but the invalid config must not be *stored*
as in-use either.

Implemented in `KissHardware.cpp` `set_alias()` via a new
`Alias::is_unrepresentable_nN()` predicate in KissTypes.hpp
(`class_mask() != 0 && callsign_len() == 6`), covered by three explicit tests
(`alias_unrepresentable_nN_6char_prefix`, `alias_representable_nN_short_prefix`,
`alias_explicit_never_unrepresentable`).

## 2. Deficiency analysis -- what the failures reveal

Re-derived from the reference source (aprsroute.hpp try_explicit_route /
find_unused_router_address_index) and the actual routes.json vectors, not
from the handoff's bucket labels.

### A. Self-routing is not implemented (17 cases)

The core digipeater operation: **the first unused address is our own
callsign -> route it** (mark H-bit; insert our callsign when the matched
address is not already ours).  `can_repeat()` uses mycall ONLY for
rejection (dest==mycall, src==mycall, mycall-in-path) -- never as a
route target.

Cases: 76-83, 84, 85, 89, 90, 94, 102, 147, 201, 215.

- 76-83: mycall at the first unused slot at each path position after a
  used prefix (DIGIA*,DIGIB,... full-path variants). 76 = full path
  (8 addrs) -> mark only.
- 84: DIGI,ROUTE -> DIGI*.
- 85: CALL*,DIGI -> CALL,DIGI*.
- 89: RELAY*,DIGI -> RELAY,DIGI*.
- 90/94/102/147: mycall first-unused, mark only.
- 201/215: mycall with SSID (DIGI1-1 == mycall with ssid=1) at
  first-unused -> mark.  Requires SSID-aware mycall matching (already in
  match_shifted_callsign check_ssid).

Required rule: during the first-unused scan, an address that equals
mycall (SSID-aware) at the first-unused position is a match; mark it;
insert before it unless it is already our own address (pass-through).
Reference confirms: find_unused_router_address_index matches the router
address itself.

### B. Explicit-alias matching: SSID ignored + exact-match not enforced (9 cases)

`alias_call_matches()` explicit branch matches the 6-char callsign only.
The reference is SSID-sensitive: an explicit alias "DIGI" does NOT match
"DIGI-1".  The reference's find_unused_router_address_index requires the
matched address to be UNMARKED, and for explicit matches the full
address (callsign+SSID) must equal.

Cases: 71 (WIDE8 vs WIDE8-2), 91, 105, 107, 114, 116, 119 (RELAY vs
RELAY-1), 211 (WIDE1 vs WIDE1-3), 218 (DIGI vs DIGI-1).

- 91 is the odd one: mycall=ROUTER, packet `DIGI>APRS,DIGI,ROUTER` with
  explicit alias DIGI at index 0.  The reference routes it
  (`ROUTER,DIGI*,ROUTER`); our can_repeat rejects because the mycall-in-path
  guard fires (ROUTER at index 1, not first-unused).  The mycall-in-path
  rejection must not suppress an explicit-alias route.  Same root fix:
  the preempt-era mycall-in-path guard needs re-examination for explicit
  routing.
- 105/107 (WIDE*,WIDE,WIDE with explicit alias WIDE): reference does
  NOT route because the *first* matching address (index 0) is marked
  (the reference searches for an unmarked router address; a marked match
  terminates the search). So explicit matching must also REQUIRE the
  matched address to be unmarked. These two cases are this deficiency,
  not the blocking rule (the blocking rule already drops them -- the
  problem is we route before the blocking rule can see the marked address).

### C. Skip_complete (6 cases)

- 160, 162, 163, 164, 166: an exhausted n-N alias (SSID==0, H-bit unset)
  at the first-unused position is treated by blocked_by_unused_address as
  a blocker -> drop.  With skip_complete, an exhausted alias is
  CONSUMABLE: mark it used and continue scanning (Decision 2 semantics,
  not yet implemented).
- 165: skip_complete removes the exhausted address (rewrite drops it);
  the reference KEEPS it and sets the H-bit -> `WIDE1,WIDE2,DIGI*` vs our
  `WIDE1,DIGI*`.

### D. Full-path explicit substitution (1 case)

197: full 8-addr path, explicit alias A at index 0 -> reference erases A
and substitutes (no room to insert): `DIGI*,B,C,D,E,F,G,I`. Our firmware
marks A (ignores SSID/aliasing) and would also fail on a full-path
insert.  Needs the "no room -> erase + substitute" path.

### E. Harness limitations (3 cases -- NOT firmware deficiencies)

- 169/170: `path: "FOOBAR2,FOOBAR3"` -> n-N prefix "FOOBAR" is 6 chars;
  a 6-char prefix + class digit cannot fit in an AX.25 6-char callsign
  field.  The reference (10-char string addresses) can hold it; the
  byte-level harness cannot.  The generator must detect this and mark
  the vector SKIP (unrepresentable) -- same class as the 114 skips for
  unsupported modes.  NOT a routing bug: on a real wire, "FOOBAR2-2"
  does not exist.
- 231: `CALL--1` (negative SSID): encode_call does
  `static_cast<uint8_t>(std::stoi("-1"))` = 0xFF, then `(0xFF << 1)` =
  0xFE -- bit 7 (H-bit) set by accident.  The harness must reject SSIDs
  outside 0..15 in encode_call/parse_ax25_packet; with a correct
  encoding (SSID 15 -> 0x1E, no H-bit), the packet is properly blocked
  by the malformed-path rule and the vector passes.  Harness encoding
  bug, not a routing bug.
- 213: passes BY ACCIDENT -- explicit alias "DIGI" matches the wire
  "DIGI-7" only because SSID is ignored (deficiency B).  Once B lands,
  check 213's expected behavior (reference routes it as
  `DIGI-7*` -- the router address with SSID 7 is the match).  Update
  expectation accordingly.

## 3. Coverage matrix (what's tested / not tested)

Well-covered today:
- Blocking rule (Decision 1): many vectors + explicit suite.
- Malformed-path policy (221).
- N > n firm requirement (6 divergence vectors + 4 explicit tests).
- n-N decrement / insert / substitute (Patterns A-D).

NOT covered (must be added, per section 1-2):
- Explicit alias with SSID (positive: "DIGI-7" matches alias "DIGI-7";
  negative: "DIGI-7" does not match "DIGI").
- mycall as the route target (self-routing) -- zero positive vectors
  today; only the 17 failure cases.
- Operator-classification cross-product (RELAY-as-n-N, WIDE-as-explicit).
- skip_complete mark-and-keep: no passing assertion of the intended
  behavior (only failing vectors).
- 6-char n-N prefix boundary (mark SKIP, not silent truncation).
- Negative/invalid SSID encoding (assert rejection in encode_call).

## 4. Implementation order

1. **Harness + classifier injection** (section 1) -- data-path change,
   no firmware impact. Prerequisite for everything else.
   Includes: encode_call SSID range check; SKIP-marking for 6-char
   prefix n-N aliases; regression check (totals unchanged).
2. **Self-routing** (deficiency A) -- DONE (commit follows).  Added
   mycall-as-route-target in the scan.  Closed 18 cases: the 17 failing
   cases (76-83, 84, 85, 89, 90, 94, 102, 147, 201, 215) PLUS the
   diverging case 87 (`DIGI,WIDE1-1` -> `DIGI*,WIDE1-1`), and 107.
   Implementation notes:
   - Self-routing = first-unused address equals mycall (SSID-aware) ->
     mark H-bit, no insert (pass-through).  rewrite_frame already handled
     the "matched is mycall" path; only can_repeat needed the match.
   - The mycall-in-path guard now uses "first address after the last used
     address" (was "first unmarked from offset 14"), matching the
     reference's find_unused_router_address_index.  This also fixed 107.
   - ~~Self-routing is suppressed when the last-used address matches a
     configured explicit alias (marked first-match suppresses; cases
     58/59/95).~~ RETRACTED under Option A (2026-08-21): that suppression
     replicated libaprsroute's "marked-first-match suppresses" rule, which
     has no basis in the APRS spec (Digipeater Algorithm sec. 4.1 acts on
     the FIRST UNUSED address only; Direwolf implements exactly this).
3. **Option A reclassification + SSID-exact aliases** (deficiency B,
   DONE): Rob's ruling -- where libaprsroute diverges from Direwolf /
   the APRS spec on a real interoperability question, side with Direwolf.
   Changes:
   - Explicit alias matching is now SSID-exact (spec sec. 2.2: "The
     address alias includes any SSID").  Fixes 71 (`WIDE8` vs `WIDE8-2`),
     211 (`WIDE1` vs `WIDE1-3`), 218 (`DIGI` vs `DIGI-1`).  Test
     can_repeat_exact_callsign_match updated to assert SSID mismatch is
     rejected.
   - Harness H-bit encoding fixed to prefix-closure semantics (TNC-2
     display convention: '*' marks only the LAST used digi, all earlier
     path addresses are IMPLIED used on the wire).  This moved 58/95 from
     accidental-pass to genuine Option-A divergences and exposed 193/210
     as reference SSID-insensitivity cases.
   - Documented divergences (reference wrong per spec, firmware right):
     58, 59, 91, 95, 105 (last-used search / marked-first-match
     suppression), 193, 210 (explicit-alias SSID insensitivity).
4. **skip_complete mark-and-keep** (deficiency C, 6 cases): exhausted
   alias consumed (mark H, keep in path), per Decision 2.  Remaining
   conformance failures 160, 162-166 are exactly this mode plus the
   substitute interaction (165).
5. **Full update-array rewrite** (existing 2026-07-31 plan) -- mechanical
   unification; expected residual: substitution rewrite (197) and
   Pattern C/D substitution divergences only.

Current status after step 3: conformance 113 passed / 20 documented
divergences / 7 failed / 116 skipped; smoke 9/9; explicit 32/0.  The 7
failures are feature gaps (steps 4-5), not routing-model defects.

After each step: run all three suites, report exact delta.  Any
previously-passing test that fails = stop and report.

## Verification plan

- Baseline (this session): conformance 92 passed / 14 diverged / 36
  failed / 114 skipped; smoke 9/9; explicit 24/7.
- After each step: same runner; delta must equal the step's target case
  count, nothing else.
- End state: supported-mode failures = 0 (36 -> 0); divergences =
  documented (7 Pattern C/D + 6 N>n + 221 + H-bit display).

## Key files

  tests/gen_conformance_data.py      -- classifier injection + skips
  tests/test_routes_data.hpp          -- regenerated data
  tests/test_digipeater_conformance.cpp -- runner (uses new cls field)
  tests/test_digipeater_harness.hpp   -- encode_call SSID range check
  Core/TNC/DigipeaterCore.hpp         -- self-routing, explicit SSID,
                                         skip_complete, full-path sub
  docs/plans/2026-07-31-routing-update-array.md -- update-array design
  HANDOFF.md                          -- working note (updated)
