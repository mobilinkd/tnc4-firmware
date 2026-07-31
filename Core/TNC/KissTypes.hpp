// Copyright 2026 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.
//
// POD types and constants shared between firmware and host-side tests.
// Zero firmware dependencies -- only standard C++ headers.

#ifndef MOBILINKD__TNC__KISSTYPES_HPP_
#define MOBILINKD__TNC__KISSTYPES_HPP_

#include <array>
#include <cstdint>
#include <cstddef>

namespace mobilinkd { namespace tnc { namespace kiss {

const size_t CALLSIGN_LEN = 8;

/**
 * AX.25 callsign with SSID.  Same 8-byte footprint as the old
 * std::array<char, 8>, but now carries the SSID explicitly.
 *
 * Layout (matches EEPROM wire format byte-for-byte):
 *   [0..5]  callsign characters, space-padded (NOT NUL-padded)
 *   [6]     pad, must be 0
 *   [7]     SSID (0-15)
 *
 * Space-padding (not NUL) matches the AX.25 on-air encoding where
 * unused character positions are shifted spaces (0x40).
 */
struct call_t {
    std::array<char, 6> callsign{};
    uint8_t pad = 0;
    uint8_t ssid = 0;
};
static_assert(sizeof(call_t) == 8, "call_t must be 8 bytes for EEPROM compatibility");

struct Alias {
    call_t call;                ///< Callsign with SSID.
    bool set;                   ///< Alias is configured.
    bool use;                   ///< Use this alias.
    uint8_t hops;               ///< Hop count remaining
}; // size = 11

const size_t BEACON_TEXT_LEN = 128;
const size_t BEACON_MAX_PATH_ADDRS = 4;

/**
 * Beacon configuration stored in EEPROM.
 *
 * The path is stored as pre-encoded AX.25 addresses (6 shifted bytes + 1 SSID
 * byte each) so that the beacon timer callback can construct the frame with
 * pure byte operations and no string parsing.
 *
 * Each path address byte layout:
 *   [0..5]: ASCII callsign << 1, space-padded to 6 chars
 *   [6]:    (ssid << 1) | flags
 *           flags: bit 7 (H-bit) = 0 (we're initiating, not repeating)
 *                  bit 0 (C-bit) = 0 (set during frame construction)
 */
struct Beacon {
    call_t dest;                            ///< Callsign with SSID.
    uint8_t dest_len;                       ///< Length of dest string (0-8).
    uint8_t path[BEACON_MAX_PATH_ADDRS][7]; ///< Pre-encoded AX.25 path addresses.
    uint8_t path_count;                     ///< Number of addresses in path (0-4).
    uint8_t text_len;                       ///< Actual number of bytes in text[].
    uint8_t text[BEACON_TEXT_LEN];          ///< Beacon payload text.
    uint16_t seconds;                       ///< Number of seconds between beacons.
}; // size = 8 + 1 + 28 + 1 + 1 + 128 + 2 = 169

const size_t NUMBER_OF_ALIASES = 8;     // 80 bytes
const size_t NUMBER_OF_BEACONS = 4;     // 672 bytes

namespace hardware {

// Preemptive digipeating modes -- EXPLICITLY UNSUPPORTED.
//
// The APRS spec (WB4APR, preemptive-digipeating.txt) defines DROP and MARK,
// but the two spec documents (preemptive-digipeating.txt and RR-bits.txt)
// disagree on RR-bit handling.  No implementation follows the spec exactly:
// Direwolf ignores RR bits, implements DROP/MARK/TRACE, and deprecates DROP
// and MARK in favor of TRACE.  The community considers the feature unreliable.
// There is no formal, ratified standard for preemptive digipeating.
//
// These constants are retained for documentation and future use, but the
// firmware does not implement any preemptive routing behavior.  The routing_mode
// bits 0x01-0x08 are reserved and ignored by the routing engine.
constexpr uint8_t ROUTING_PREEMPT_FRONT    = 0x01; // Reserved -- not implemented
constexpr uint8_t ROUTING_PREEMPT_TRUNCATE = 0x02; // Reserved -- not implemented
constexpr uint8_t ROUTING_PREEMPT_DROP     = 0x04; // Reserved -- not implemented
constexpr uint8_t ROUTING_PREEMPT_MARK     = 0x08; // Reserved -- not implemented
constexpr uint8_t ROUTING_SKIP_COMPLETE    = 0x80;

// Substitution of exhausted n-N aliases (SSID decremented to 0) is hardcoded,
// not configurable.  Direwolf does this unconditionally -- there is no toggle.
// The alternative (leaving WIDEn-0 in the path) produces dead addresses that
// every downstream digi must skip.  No implementation in the field runs
// without substitution.

} // namespace hardware

}}} // mobilinkd::tnc::kiss

#endif // MOBILINKD__TNC__KISSTYPES_HPP_
