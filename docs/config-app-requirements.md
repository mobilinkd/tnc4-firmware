# Firmware Protocol Reference: Digipeater & Beacon Configuration

## Overview

This is the firmware-side KISS protocol reference for configuring the TNC4
digipeater and beacon features.  It is the contract that the iOS, Android, and
Python (`tnc1-python-config`) config apps build against.

All commands below are implemented and live on branch
`feature/digipeater-integration`.  Handlers are in `Core/TNC/KissHardware.cpp`
(`handle_ext_request()` and the getter/setter helpers); the command opcodes and
data structures are declared in `Core/TNC/KissHardware.hpp` and
`Core/TNC/KissTypes.hpp`.

All multi-byte integers are big-endian (network byte order) unless noted.

## Feature discovery (GET_ALL_VALUES)

On connect, the config app issues `GET_ALL_VALUES`.  The firmware includes the
following in the response stream so the app can detect digipeater/beacon
support and read the current settings without extra round-trips
(KissHardware.cpp:866-875):

| Response | Payload | Meaning |
|----------|---------|---------|
| EXT_GET_ALIASES (0xC1 0x88) | 1 byte | Number of alias slots (currently 8) |
| EXT_GET_BEACON_SLOTS (0xC1 0x8C) | 1 byte | Number of beacon slots (currently 4) |
| EXT_GET_DIGIPEATER (0xC1 0x8B) | 3 bytes | `enabled`, `routing_mode`, `dedupe_seconds` |

The presence of the EXT_GET_ALIASES / EXT_GET_BEACON_SLOTS responses is the
signal that the connected firmware supports these features.  These responses
are emitted before GET_DATETIME, which must remain the last item in the stream
(the iOS app depends on that ordering).

GET_ALL_VALUES does **not** include the full alias or beacon lists.  To fetch
those, issue the bulk commands below.

## Command reference

| Command | Bytes | Direction | Payload / Reply |
|---------|-------|-----------|-----------------|
| EXT_GET_ALIASES | 0xC1 0x88 | Host→TNC | Reply: alias count (1 byte) |
| EXT_GET_ALIAS | 0xC1 0x89 | Host→TNC | Request: index (1 byte). Reply: see Alias reply below |
| EXT_SET_ALIAS | 0xC1 0x8A | Host→TNC | Request: index (1 byte) + alias (11 bytes). Reply: EXT_OK |
| EXT_GET_DIGIPEATER | 0xC1 0x8B | Host→TNC | Reply: 3 bytes (enabled, routing_mode, dedupe_seconds) |
| EXT_GET_BEACON_SLOTS | 0xC1 0x8C | Host→TNC | Reply: beacon slot count (1 byte) |
| EXT_GET_BEACON | 0xC1 0x8D | Host→TNC | Request: slot (1 byte). Reply: see Beacon reply below |
| EXT_SET_BEACON | 0xC1 0x8E | Host→TNC | Request: see Beacon request below. Reply: EXT_OK |
| EXT_SET_DIGIPEATER | 0xC1 0x8F | Host→TNC | Request: 3 bytes (enabled, routing_mode, dedupe_seconds). Reply: EXT_OK |
| EXT_GET_ALL_DIGIPEATER_CONFIGS | 0xC1 0x90 | Host→TNC | Reply: digipeater settings + all aliases (bulk) |
| EXT_GET_ALL_BEACON_CONFIGS | 0xC1 0x91 | Host→TNC | Reply: all beacon configs (bulk) |

## Data structures

### call_t (8 bytes)

A callsign with SSID, stored fixed-width for EEPROM compatibility
(KissTypes.hpp:30):

| Offset | Size | Field |
|--------|------|-------|
| 0 | 6 | callsign -- ASCII, space-padded (NOT AX.25 shifted) |
| 6 | 1 | pad (0) |
| 7 | 1 | ssid (0-15) |

`static_assert(sizeof(call_t) == 8)`.

### Alias reply (EXT_GET_ALIAS, 12 bytes)

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | alias index |
| 1 | 8 | call_t |
| 9 | 1 | set (0/1) |
| 10 | 1 | use (0/1) |
| 11 | 1 | hops (1-15) |

### Alias request (EXT_SET_ALIAS, 12 bytes)

Same layout as the reply: index (1) + call_t (8) + set (1) + use (1) + hops (1).

### Digipeater settings (3 bytes)

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | digipeater_enabled (0/1) |
| 1 | 1 | routing_mode (bitmask, see below) |
| 2 | 1 | dedupe_seconds |

Used by both EXT_GET_DIGIPEATER (reply) and EXT_SET_DIGIPEATER (request).

> Note: the opcode comments in KissHardware.hpp describe these as "5 bytes";
> that comment is stale.  The handler reads and replies exactly 3 bytes.

### Beacon request (EXT_SET_BEACON, variable)

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | beacon slot |
| 1 | 2 | interval seconds (big-endian) |
| 3 | var | destination callsign string, NUL-terminated |
| ... | var | path string, NUL-terminated (e.g. "WIDE1-1,WIDE2-2") |
| ... | var | beacon text, NUL-terminated |

The firmware parses the path string into pre-encoded AX.25 addresses
(max 4) for storage.

### Beacon reply (EXT_GET_BEACON)

Same field order as the request: slot (1) + interval (2) + dest (NUL) +
path (NUL) + text (NUL).

## Bulk commands

### EXT_GET_ALL_DIGIPEATER_CONFIGS (0xC1 0x90)

Single response containing the digipeater settings followed by every alias:

```
[0xC1][0x90]
[enabled(1)][routing_mode(1)][dedupe_seconds(1)]
for each of NUMBER_OF_ALIASES (8):
    [index(1)][call_t(8)][set(1)][use(1)][hops(1)]   = 12 bytes
```

Total: 2 + 3 + 8×12 = 101 bytes.

### EXT_GET_ALL_BEACON_CONFIGS (0xC1 0x91)

Single response containing every beacon, variable length:

```
[0xC1][0x91]
for each of NUMBER_OF_BEACONS (4):
    [slot(1)][interval_H(1)][interval_L(1)]
    [dest (NUL-terminated)]
    [path (NUL-terminated)]
    [text (NUL-terminated)]
```

The firmware computes the exact size in a first pass before emitting, so the
response is never over-allocated.

## Routing mode flags

`routing_mode` is a bitmask (KissTypes.hpp:76-81):

| Flag | Value | Description | Implemented |
|------|-------|-------------|-------------|
| ROUTING_PREEMPT_FRONT | 0x01 | On alias-scan miss, find mycall in path, mark H-bit, truncate after it | Yes |
| ROUTING_PREEMPT_TRUNCATE | 0x02 | Move mycall behind last used, erase middle | No |
| ROUTING_PREEMPT_DROP | 0x04 | Erase all addresses in front of ours | No |
| ROUTING_PREEMPT_MARK | 0x08 | Mark our address used, leave path as-is | No |
| ROUTING_SKIP_COMPLETE | 0x80 | Drop already-completed addresses from the path | Yes |

Bits 0x10, 0x20, and 0x40 are reserved (unused).

Substitution of exhausted n-N aliases is hardcoded, not configurable.
Direwolf unconditionally replaces an exhausted n-N alias (SSID decremented
to 0) with the digipeater's callsign.  No implementation in the field runs
without substitution.

The config app may expose SKIP_COMPLETE as a toggle.  The preempt bits
(0x01-0x08) are reserved and ignored.

## Alias auto-classification

When an alias is configured, its callsign prefix determines routing behavior:
prefixes WIDE, TRACE, RELAY, ECHO, GATE, and TEMP are treated as n-N aliases
(SSID decrement + H-bit marking); anything else is treated as an explicit
(substitute) alias.
