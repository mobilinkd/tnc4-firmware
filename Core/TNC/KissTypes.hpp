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

/**
 * A digipeater alias.  Classification (n-N vs explicit) is operator-marked
 * through the class mask rather than inferred from a prefix list.  See
 * ADR-0008.
 *
 * - n-N alias: the class mask (bits 1-7) is non-zero.  `call.callsign` is
 *   the routing prefix (e.g. "WIDE"); bit n serves class n (n = 1..7).
 * - explicit alias: the class mask is zero.  `call` is matched by full
 *   callsign equality.
 *
 * A slot is "configured" when its callsign is non-empty (first byte is
 * neither NUL nor space).
 */
struct Alias {
    call_t call;                ///< Routing prefix (n-N) or full callsign (explicit).
    uint8_t flags;              ///< bit 0 = use; bits 1-7 = class mask.

    static constexpr uint8_t USE = 0x01;         ///< bit 0: alias is enabled.
    static constexpr uint8_t CLASS_MASK = 0xFE;  ///< bits 1-7: served classes.

    /// True when the slot is configured (callsign is non-empty).
    bool is_configured() const {
        return call.callsign[0] != '\0' && call.callsign[0] != ' ';
    }
    /// True when the alias is enabled for use.
    bool in_use() const { return flags & USE; }
    /// The class mask: bits 1-7 select which n-N classes (n=1..7) are served.
    uint8_t class_mask() const { return flags & CLASS_MASK; }
    /// True when the alias is an n-N alias (serves at least one class).
    bool is_nN() const { return class_mask() != 0; }
    /// Number of leading non-space, non-NUL characters in the callsign field.
    uint8_t callsign_len() const {
        uint8_t n = 0;
        while (n < 6 && call.callsign[n] != '\0' && call.callsign[n] != ' ')
            ++n;
        return n;
    }
    /// True when this alias is unrepresentable on the wire: an n-N alias
    /// whose routing prefix fills all 6 callsign positions, leaving no room
    /// for the class digit (PREFIXn needs 7 wire characters).  The KISS
    /// set-alias command must zero the config byte for such an alias.
    bool is_unrepresentable_nN() const {
        return is_nN() && callsign_len() == 6;
    }
}; // size = 9
static_assert(sizeof(Alias) == 9, "Alias must be 9 bytes for EEPROM compatibility");

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
}; // size = 8 + 1 + 28 + 1 + 1 + 128 + 2 = 170 (incl. 1 byte padding before seconds)

const size_t NUMBER_OF_ALIASES = 8;     // 72 bytes (9 each)
const size_t NUMBER_OF_BEACONS = 4;     // 680 bytes (170 each)

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
