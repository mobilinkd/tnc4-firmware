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
using call_t = std::array<char, CALLSIGN_LEN>;

struct Alias {
    call_t call;                ///< Callsign.  Pad unused with NUL.
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
    call_t dest;                            ///< callsign.  Pad unused with NUL.
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

// Routing mode flags for digipeater: only one of PREEMPT_FRONT/TRUNCATE/DROP/MARK may be set.
constexpr uint8_t ROUTING_PREEMPT_FRONT    = 0x01;
constexpr uint8_t ROUTING_PREEMPT_TRUNCATE = 0x02;
constexpr uint8_t ROUTING_PREEMPT_DROP     = 0x04;
constexpr uint8_t ROUTING_PREEMPT_MARK     = 0x08;
constexpr uint8_t ROUTING_SUBSTITUTE       = 0x40;
constexpr uint8_t ROUTING_SKIP_COMPLETE    = 0x80;

} // namespace hardware

}}} // mobilinkd::tnc::kiss

#endif // MOBILINKD__TNC__KISSTYPES_HPP_
