// Host-side digipeater test harness.
//
// Includes the REAL routing core (DigipeaterCore.hpp) with a test policy.
// No firmware headers, no ARM dependencies, no code duplication.
//
// Test helpers (parse_ax25_packet, encode_call, make_alias, etc.) remain
// here -- they don't duplicate any firmware code.

#pragma once

#include <array>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <optional>

// Include the real routing core -- zero firmware dependencies.
#include "Core/TNC/DigipeaterCore.hpp"

// ============================================================================
// Test policy and config
// ============================================================================

namespace test {

using namespace mobilinkd::tnc;
using namespace mobilinkd::tnc::kiss;

/**
 * Minimal config struct for testing.  Mirrors the fields of kiss::Hardware
 * that the routing core actually reads.
 */
struct TestConfig {
    call_t mycall{};
    uint8_t digipeater_enabled = 0;
    uint8_t routing_mode = 0;
    uint8_t dedupe_seconds = 30;
    Alias aliases[NUMBER_OF_ALIASES]{};
    Beacon beacons[NUMBER_OF_BEACONS]{};
};

/**
 * Test platform policy: injectable monotonic clock.
 */
struct TestPolicy {
    uint32_t tick_ = 0;
    uint32_t now_ms() { return tick_; }
};

/// The concrete digipeater type used in tests.
using TestDigipeater = DigipeaterCore<TestPolicy, TestConfig>;

// ============================================================================
// Test helper functions
// ============================================================================

// Encode a single AX.25 call string (e.g., "WIDE1" or "WIDE1-1") to 7 bytes.
// Returns [6 shifted chars][SSID byte with H-bit=0, C-bit=0].
inline std::array<uint8_t, 7> encode_call(const std::string& call)
{
    std::array<uint8_t, 7> result{};
    // Split on '-' for SSID
    std::string callsign = call;
    uint8_t ssid = 0;
    auto dash = call.find('-');
    if (dash != std::string::npos) {
        callsign = call.substr(0, dash);
        ssid = static_cast<uint8_t>(std::stoi(call.substr(dash + 1)));
    }
    // Encode callsign: 6 chars, shifted left 1, space-padded
    for (size_t i = 0; i < 6; i++) {
        result[i] = static_cast<uint8_t>((i < callsign.size() ? callsign[i] : ' ') << 1);
    }
    // SSID byte: (ssid << 1), H-bit=0, C-bit=0
    result[6] = (ssid << 1);
    return result;
}

// Parse an APRS packet string like "N0CALL>APRS,WIDE1-1,WIDE2-2:data"
// into a raw AX.25 byte array.
inline std::vector<uint8_t> parse_ax25_packet(const std::string& packet)
{
    std::vector<uint8_t> result;
    // Find > separator: src>dest
    auto gt = packet.find('>');
    if (gt == std::string::npos) return result;

    std::string src = packet.substr(0, gt);
    // Find : separator: path:info
    auto colon = packet.find(':', gt);
    if (colon == std::string::npos) return result;

    std::string path_part = packet.substr(gt + 1, colon - gt - 1);
    std::string info = packet.substr(colon + 1);

    // Encode destination
    auto dest_path = path_part;
    auto comma = dest_path.find(',');
    std::string dest = (comma != std::string::npos)
        ? dest_path.substr(0, comma) : dest_path;

    auto dest_bytes = encode_call(dest);
    result.insert(result.end(), dest_bytes.begin(), dest_bytes.end());

    // Encode source (no SSID typically)
    auto src_bytes = encode_call(src);
    result.insert(result.end(), src_bytes.begin(), src_bytes.end());

    // Encode digipeater path
    std::vector<std::string> path;
    if (comma != std::string::npos) {
        std::string remaining = dest_path.substr(comma + 1);
        size_t start = 0;
        for (size_t i = 0; i <= remaining.size(); i++) {
            if (i == remaining.size() || remaining[i] == ',') {
                path.push_back(remaining.substr(start, i - start));
                start = i + 1;
            }
        }
    }

    // Set H-bit on addresses marked with *
    for (size_t pi = 0; pi < path.size(); pi++) {
        bool has_star = false;
        std::string clean = path[pi];
        if (!clean.empty() && clean.back() == '*') {
            has_star = true;
            clean.pop_back();
        }
        auto bytes = encode_call(clean);
        if (has_star) bytes[6] |= 0x80;  // Set H-bit
        // Set C-bit on the last address
        if (pi == path.size() - 1) bytes[6] |= 0x01;
        result.insert(result.end(), bytes.begin(), bytes.end());
    }

    // Control byte: UI frame (0x03)
    result.push_back(0x03);
    // PID: No layer 3 (0xF0)
    result.push_back(0xF0);
    // Info field
    for (char c : info) result.push_back(static_cast<uint8_t>(c));

    return result;
}

// Convert an AX.25 byte buffer back to an ASCII packet string for debug output.
//
// AX.25 wire order is dest(7) + src(7) + digi*(7) + ctrl + pid + info, but the
// conventional APRS text form is SRC>DEST,DIGI1,DIGI2*:info -- so source and
// destination are emitted in reverse of their wire positions.  SSID is decoded
// on every address (source, destination, and digipeaters).
inline std::string ax25_packet_to_string(const uint8_t* buf, size_t len)
{
    if (len < 14) return "(too short)";

    // Decode one 7-byte address field (6 shifted chars + SSID byte) to "CALL"
    // or "CALL-N".  Trailing spaces are dropped; SSID 0 is omitted.
    auto decode_call = [buf](size_t addr_offset) {
        std::string call;
        for (size_t i = 0; i < 6; i++) {
            char c = static_cast<char>(buf[addr_offset + i] >> 1);
            if (c != ' ') call += c;
        }
        uint8_t ssid = (buf[addr_offset + 6] >> 1) & 0x0F;
        if (ssid > 0) call += "-" + std::to_string(ssid);
        return call;
    };

    std::string result;
    // Source lives at bytes 7-13 on the wire but prints first in APRS text.
    result += decode_call(7);
    result += '>';
    // Destination lives at bytes 0-6 on the wire but prints after the '>'.
    result += decode_call(0);

    // Digipeater path starts at byte 14; each address is 7 bytes and the last
    // one has the C-bit (bit 0) set.  Every digipeater is comma-separated from
    // what precedes it, including the first (which follows the destination).
    size_t pos = 14;
    while (pos + 7 <= len) {
        result += ',';
        for (size_t i = 0; i < 6; i++) {
            char c = static_cast<char>(buf[pos + i] >> 1);
            if (c != ' ') result += c;
        }
        uint8_t ssid_byte = buf[pos + 6];
        uint8_t ssid = (ssid_byte >> 1) & 0x0F;
        bool h_bit = (ssid_byte & 0x80) != 0;
        bool c_bit = (ssid_byte & 0x01) != 0;
        if (ssid > 0) result += '-' + std::to_string(ssid);
        if (h_bit) result += '*';
        pos += 7;
        if (c_bit) break;
    }
    result += ':';
    // Decode info (skip control + PID)
    pos += 2; // skip control and PID
    for (size_t i = pos; i < len; i++) {
        char c = static_cast<char>(buf[i]);
        if (c >= 0x20 && c < 0x7F) result += c;
    }
    return result;
}

inline std::string ax25_packet_to_string(const std::vector<uint8_t>& buf, size_t len)
{
    return ax25_packet_to_string(buf.data(), len);
}

// Create an alias with NUL-padded callsign
inline Alias make_alias(const std::string& call, uint8_t hops, bool set = true, bool use = true)
{
    Alias a{};
    a.set = set;
    a.use = use;
    a.hops = hops;
    for (size_t i = 0; i < call.size() && i < 6; i++)
        a.call.callsign[i] = call[i];
    return a;
}

// Setup a TestConfig with a specific mycall and routing mode
inline TestConfig make_test_config(const std::string& mycall,
                                   uint8_t routing_mode = hardware::ROUTING_SUBSTITUTE)
{
    TestConfig cfg{};
    cfg.digipeater_enabled = 1;
    cfg.routing_mode = routing_mode;
    cfg.dedupe_seconds = 30;
    // Parse "CALL" or "CALL-N" into callsign + ssid.
    std::string base = mycall;
    auto dash = mycall.find('-');
    if (dash != std::string::npos) {
        base = mycall.substr(0, dash);
        cfg.mycall.ssid = static_cast<uint8_t>(std::stoi(mycall.substr(dash + 1)));
    }
    for (size_t i = 0; i < base.size() && i < 6; i++)
        cfg.mycall.callsign[i] = base[i];
    return cfg;
}

} // namespace test
