# Grill-with-Docs: Digipeater Implementation — Resolved Questions

Session date: 2026-05-25. All questions resolved. CONTEXT.md updated inline.

---

## Resolved

### 1. ADR-0002 / Code Contradiction
ADR accepted but not yet implemented. Alias struct, KISS protocol commands, config app protocol, and EEPROM layout all need updating.

### 2. IoFrame Reference Counting
Add `uint8_t ref_count` to IoFrame (1 byte × 48 frames = 48 bytes). `release()` decrements, returns to pool at 0. `add_ref()` increments. Critical section protected.

### 3. TX Confirmation
Deferred — separate feature. Modulator interface needs callback mechanism (TBD).

### 4. Digipeater fork location
In IOEventTask after host write. Add ref to frame (refcount→2), process inline (linear buffer copy + route + re-queue), release incoming frame when digipeater is done.

### 5. Linear buffer size
Confirmed: 330 bytes = 10 addresses × 7 (TOCALL + MYCALL + 8 path) + 2 protocol + 256 info + 2 CRC.

### 6. Dedupe buffer lifecycle
Cleared explicitly during IOEventTask initialization. SRAM retained during Stop2 — only clears on cold POR/reset. No special RAM sections needed.

### 7. Beacon scheduling
4 FreeRTOS software timers (cheaper than dedicated task: 320 bytes vs 632+ bytes). Timer infrastructure already allocated. One timer per beacon slot. Callback constructs frame and posts to modulator queue.

### 8. libaprsroute placement
`lib/libaprsroute/` as git submodule at repo root. Boost and Blaze also to become git submodules under `lib/`.

### 9. Memory model: segmented → linear → segmented
1. Add ref to incoming IoFrame (refcount→2)
2. Copy to linear buffer, route via libaprsroute
3. Copy to new IoFrame from pool
4. Release incoming frame (decref, digipeater done)
5. Post new frame to modulator queue
6. OOM at any step → release + log (debug builds only, SWO)

### 10. PTT/CSMA arbitration
FIFO queue with digipeated frames posted to front. TX queue drains in burst when channel clears. Digipeated frames use p=0 CSMA; beacons/user use standard p-persist.

### 11. Digipeater enable/disable
Dedicated boolean `digipeater_enabled` in Hardware struct, EEPROM-persisted, default false.

### 12. CubeMX project structure
Project defined by `tnc4-firmware.ioc`. CubeMX generates Core/, Middlewares/, Drivers/. User code must stay within template tags. `Core/TNC/` should move to top-level `TNC/`.

### 13. KISS protocol commands needed
`EXT_GET/SET_ROUTING_MODE`, `EXT_GET/SET_DEDUPE_SECONDS`, `EXT_GET/SET_DIGIPEATER_ENABLE`. Existing alias commands need update per ADR-0002.
