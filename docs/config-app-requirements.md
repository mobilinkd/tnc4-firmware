# Firmware Requirements: Config App Digipeater & Beacon Support

## Overview

The `tnc1-python-config` app (feature/RFCOMM branch) needs to configure the
TNC4 digipeater and beacon features.  This document captures the firmware-side
changes required to support that.

## Current State

The digipeater and beacon KISS commands already exist in the firmware on
branch `feature/digipeater-integration`:

| Command | Bytes | Direction | Purpose |
|---------|-------|-----------|---------|
| EXT_GET_ALIASES | 0xC1 0x88 | Host->TNC | Get alias count (1 byte reply) |
| EXT_GET_ALIAS | 0xC1 0x89 | Host->TNC | Get one alias (takes alias index) |
| EXT_SET_ALIAS | 0xC1 0x8A | Host->TNC | Set one alias (12 bytes) |
| EXT_GET_DIGIPEATER | 0xC1 0x8B | Host->TNC | Get digipeater settings (3 bytes) |
| EXT_GET_BEACON_SLOTS | 0xC1 0x8C | Host->TNC | Get beacon slot count (1 byte reply) |
| EXT_GET_BEACON | 0xC1 0x8D | Host->TNC | Get one beacon (takes slot index) |
| EXT_SET_BEACON | 0xC1 0x8E | Host->TNC | Set one beacon |
| EXT_SET_DIGIPEATER | 0xC1 0x8F | Host->TNC | Set digipeater settings (3 bytes) |

These are handled in `handle_ext_request()` (KissHardware.cpp:768) and the
getter/setter functions (KissHardware.cpp:183-370).

## Requirements

### 1. Add EXT_GET_ALIASES and EXT_GET_BEACON_SLOTS to GET_ALL_VALUES

**File:** `Core/TNC/KissHardware.cpp`, GET_ALL_VALUES case (line ~701)

Add two lines after the existing EXT_GET_MODEM_TYPES response (line ~747):

```cpp
ext_reply(hardware::EXT_GET_ALIASES, (uint8_t)NUMBER_OF_ALIASES);
ext_reply(hardware::EXT_GET_BEACON_SLOTS, (uint8_t)NUMBER_OF_BEACONS);
```

The config app uses the presence of these responses in the GET_ALL_VALUES
stream as the signal that digipeater and beacon features are supported.  No
new capability bits needed.

These must come before the GET_DATETIME response (which must remain last, per
the existing comment: "GET_DATETIME must always be last. iOS config app
depends on it.").

### 2. Add EXT_GET_DIGIPEATER to GET_ALL_VALUES

Include the current digipeater settings in GET_ALL_VALUES so the config app
gets them on connect without a separate request:

```cpp
uint8_t digi_reply[3] = {
    digipeater_enabled,
    routing_mode,
    dedupe_seconds
};
reply_ext(hardware::EXT_GET_DIGIPEATER, digi_reply, 3);
```

### 3. Fix reply_ext NUL-truncation bug

**File:** `Core/TNC/KissHardware.cpp`, `reply_ext()` (line ~158)

The current implementation stops copying data at the first NUL byte:

```cpp
// BROKEN: stops at first NUL
for (uint16_t i = 0; i != len and data[i] != 0; i++)
    buffer[i + N] = data[i];
```

This corrupts the alias response.  `get_alias()` packs 12 bytes:
`[index, call[0..7], set, use, hops]`.  The callsign is NUL-padded, so a
4-char callsign like "WIDE" has NUL at byte 5.  `reply_ext` stops there,
sending only `[index, 'W','I','D','E']` -- missing set, use, and hops
entirely.

**Fix:** Copy all `len` bytes unconditionally:

```cpp
std::copy(data, data + len, buffer + N);
```

This affects `reply_ext` only.  The `get_beacon()` function already builds
its own buffer with explicit NUL terminators and calls `ioport->write()`
directly, so it is unaffected.

### 4. Consider EXT_GET_ALL_ALIASES bulk command (optional)

Currently the config app must issue 8 separate EXT_GET_ALIAS requests after
connect to fetch all aliases.  A bulk command could return all aliases in one
response:

```
EXT_GET_ALL_ALIASES = {0xC1, 0x90}
Reply: for each alias (up to NUMBER_OF_ALIASES):
  [index(1)] [call(8)] [set(1)] [use(1)] [hops(1)]  = 12 bytes each
Total: 1 + 12*8 = 97 bytes max
```

This is optional -- the config app can work with individual requests.  But
it would reduce connect latency and simplify the app-side code.

### 5. Consider EXT_GET_ALL_BEACONS bulk command (optional)

Similarly, a bulk beacon fetch could return all 4 beacons in one response.
However, beacons contain variable-length NUL-terminated strings (dest, path,
text up to 128 bytes), so the response could be large (~600 bytes worst case).
This is feasible but less critical than the alias bulk command.

## Summary of Required Changes

| Priority | Change | File | Complexity |
|----------|--------|------|------------|
| Required | Add EXT_GET_ALIASES to GET_ALL_VALUES | KissHardware.cpp:747 | 1 line |
| Required | Add EXT_GET_BEACON_SLOTS to GET_ALL_VALUES | KissHardware.cpp:747 | 1 line |
| Required | Add EXT_GET_DIGIPEATER to GET_ALL_VALUES | KissHardware.cpp:747 | 4 lines |
| Required | Fix reply_ext NUL-truncation | KissHardware.cpp:158 | 1 line |
| Optional | EXT_GET_ALL_ALIASES bulk command | KissHardware.hpp + .cpp | ~20 lines |
| Optional | EXT_GET_ALL_BEACONS bulk command | KissHardware.hpp + .cpp | ~30 lines |

## Routing Mode Constants (reference)

For the config app UI:

| Flag | Value | Description |
|------|-------|-------------|
| ROUTING_PREEMPT_FRONT | 0x01 | Move our callsign to front of path |
| ROUTING_PREEMPT_TRUNCATE | 0x02 | Move our callsign behind last used, erase middle |
| ROUTING_PREEMPT_DROP | 0x04 | Erase all addresses in front of ours |
| ROUTING_PREEMPT_MARK | 0x08 | Mark our address as used, leave path as-is |
| ROUTING_SUBSTITUTE | 0x40 | Replace exhausted n-N address with our callsign |
| ROUTING_SKIP_COMPLETE | 0x80 | Drop completed addresses from path |

Note: PREEMPT_* modes are read from settings but not yet implemented in
rewrite_frame().  The config app will expose these controls for development
purposes.