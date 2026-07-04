# Digipeater Integration Plan — libaprsroute

## Sequencing

1. **ADR-0002: Global routing mode** — Remove per-alias `insert_id`/`preempt` booleans, add `routing_mode` byte to `Hardware` struct. Fix `EXT_GET/SET_ALIAS` KISS commands for the smaller struct. Update EEPROM layout. The KISS API version does not need to bump — nothing upstream implements the digipeater-specific commands yet.

2. **libaprsroute submodule** — Add `lib/libaprsroute/` as git submodule. Wire into CMake build (`cmake/tnc/CMakeLists.txt`). Add include path, link the static library.

3. **`can_repeat()` + `rewrite_frame()`** — Real implementations using libaprsroute:
   - `can_repeat()`: UI frame check, alias matching (set+use, hops>0), routing_mode check
   - `rewrite_frame()`: Segmented IoFrame → linear buffer → libaprsroute::route() → new IoFrame

4. **Digipeater fork in IOEventTask** — After host write, if digipeater is enabled, `add_ref()` the frame and post it to `digipeaterQueueHandle` for async processing.

5. **KISS protocol commands** — `EXT_GET/SET_ROUTING_MODE`, `EXT_GET/SET_DIGIPEATER_ENABLE`, update `EXT_GET/SET_ALIAS` for ADR-0002 struct. `EXT_GET/SET_DEDUPE_SECONDS` is added later but reserved now.

6. **Dedupe buffer** — CRC32 history table (source + dest + info), 30-second window. Implemented after libaprsroute frame parsing is available so we can extract source/dest/info from the parsed frame rather than raw HDLC bytes.

7. **Beacon scheduling** — 4 FreeRTOS software timers, callback constructs frame and posts to modulator queue. Lowest priority — can overlap with dedupe.

## Key constraints

- Static allocation only (no heap)
- Segmented IoFrame → linear 330-byte buffer → libaprsroute → new IoFrame (grill session item #9)
- Digipeated frames use p=0 CSMA (ADR-0003); beacons use standard p-persist
- Dedupe buffer cleared on IOEventTask init; survives Stop2
- Digipeater disabled by default (`digipeater_enabled` in EEPROM, default false)
