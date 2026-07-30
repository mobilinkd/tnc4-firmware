// Copyright 2026 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.
//
// Platform-independent APRS digipeater routing core.
//
// CRTP design: DigipeaterCore<Policy, Config> inherits from Policy.
// Policy must provide:
//   uint32_t now_ms()  -- monotonic millisecond timestamp
//
// Config must provide (by const reference):
//   call_t mycall
//   uint8_t digipeater_enabled
//   uint8_t routing_mode
//   uint8_t dedupe_seconds
//   Alias aliases[NUMBER_OF_ALIASES]
//   Beacon beacons[NUMBER_OF_BEACONS]
//
// No ARM headers, no IoFrame, no cmsis_os, no dynamic allocation.
// Compiles on host (x86, C++20) and target (Cortex-M4F, C++20).

#ifndef MOBILINKD__TNC__DIGIPEATERCORE_HPP_
#define MOBILINKD__TNC__DIGIPEATERCORE_HPP_

#include "KissTypes.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace mobilinkd { namespace tnc {

/**
 * APRS digipeater with deduplication and direct AX.25 buffer routing.
 *
 * Memory model: input iterator -> linear buffer -> output buffer
 * - 330-byte linear buffer (10 addresses x 7 + 2 protocol + 256 info + 2 CRC)
 * - 32-entry dedupe ring (hash of source + info)
 * - Configurable dedupe window (dedupe_seconds)
 *
 * Routing operates directly on AX.25 7-byte address blocks in linear_buf_.
 * No string conversion, no external routing library.
 */
template <typename Policy, typename Config>
struct DigipeaterCore : Policy
{
    static constexpr size_t LINEAR_BUF_SIZE = 330;
    static constexpr size_t MAX_DEDUPE_ENTRIES = 32;
    static constexpr size_t MAX_PATH_ADDRS = 8;

    const Config& cfg_;

    // Linear buffer for iterator->contiguous bridging
    std::array<uint8_t, LINEAR_BUF_SIZE> linear_buf_;
    size_t linear_len_ = 0;

    // Dedupe ring buffer
    struct DedupeEntry {
        uint32_t crc;
        uint32_t timestamp;  // now_ms() when recorded
        bool valid;
    };
    std::array<DedupeEntry, MAX_DEDUPE_ENTRIES> dedupe_ring_;
    size_t dedupe_head_ = 0;

    // Routing match state, populated by can_repeat() and consumed by
    // rewrite_frame_impl().  Tracks how the match was found so that the
    // rewrite applies the correct rule (normal alias decrement vs preempt
    // truncate-and-mark).
    enum class MatchType : uint8_t {
        None = 0,           // No match yet.
        Alias = 1,          // Normal first-unmatched alias match.
        PreemptFront = 2,   // Mycall found in path (preempt_front fallback).
    };
    MatchType match_type_ = MatchType::None;
    size_t match_addr_offset_ = 0;     // Linear-buffer offset of the matched addr.
    bool match_is_nN_ = false;         // True if matched alias is an n-N prefix.
    const kiss::Alias* match_alias_ = nullptr;  // Set only for Alias matches.

    explicit DigipeaterCore(const Config& cfg)
    : cfg_(cfg)
    {
        dedupe_ring_.fill({0, 0, false});
    }

    // ========================================================================
    // Deduplication
    // ========================================================================

    /**
     * Scan the dedupe history table and remove outdated entries.
     * Called at the top of the digipeater task loop.
     */
    void clean_history()
    {
        uint32_t now = this->now_ms();
        uint32_t dedupe_ms = static_cast<uint32_t>(cfg_.dedupe_seconds) * 1000;
        for (auto& e : dedupe_ring_) {
            if (e.valid && (now - e.timestamp) > dedupe_ms) {
                e.valid = false;
            }
        }
    }

    /**
     * Compute a hash key for deduplication over source + info fields.
     * This gives us a unique frame fingerprint without parsing the full path.
     */
    uint32_t compute_dedupe_key(const uint8_t* buf, size_t len)
    {
        // buf contains raw AX.25 frame: dest(7) + src(7) + digi(0-56) + ctrl(1) + pid(1) + info
        if (len < 15) return 0;

        // Count digipeater addresses: each is 7 bytes, last has bit 0 set in SSID
        size_t pos = 14; // dest(7) + src(7)
        size_t addr_count = 0;
        while (pos + 7 <= len && addr_count < 8) {
            addr_count++;
            if (buf[pos + 6] & 0x01) break; // last address (C-bit in SSID byte)
            pos += 7;
        }
        pos += 7; // skip past the last address
        // Now at control(1) + pid(1) + info
        const uint8_t* src = buf + 7;
        size_t info_start = pos + 2; // skip control + pid
        if (info_start > len) return 0;
        size_t info_len = len - info_start;

        // Simple hash: XOR of source + first 64 bytes of info
        uint32_t h = 0;
        for (int i = 0; i < 7; i++) h = (h << 5) - h + src[i];
        for (size_t i = 0; i < info_len && i < 64; i++)
            h = (h << 5) - h + buf[info_start + i];
        return h;
    }

    /**
     * Check if a frame has been seen recently (deduplication).
     */
    bool is_duplicate(uint32_t crc)
    {
        for (auto& e : dedupe_ring_) {
            if (e.valid && e.crc == crc) return true;
        }
        return false;
    }

    /**
     * Record a frame CRC in the dedupe ring.
     */
    void record_frame(uint32_t crc)
    {
        dedupe_ring_[dedupe_head_] = {crc, this->now_ms(), true};
        dedupe_head_ = (dedupe_head_ + 1) % MAX_DEDUPE_ENTRIES;
    }

    // ========================================================================
    // AX.25 address helpers
    // ========================================================================

    /**
     * Compare a 6-byte shifted AX.25 callsign against an unshifted ASCII string.
     * Returns true if they match (ignoring trailing spaces in the AX.25 field).
     */
    bool match_shifted_callsign(const uint8_t* shifted, const char* unshifted, size_t unshifted_len)
    {
        for (size_t i = 0; i < 6; i++) {
            char c = shifted[i] >> 1;
            if (i < unshifted_len) {
                if (c != unshifted[i]) return false;
            } else if (c != ' ') {
                return false; // Expected trailing space
            }
        }
        return true;
    }

    /**
     * Compare a 6-byte shifted AX.25 callsign against an 8-byte call_t.
     * call_t is NUL-padded -- compare up to the NUL or first 6 chars.
     */
    bool match_shifted_callsign(const uint8_t* shifted, const kiss::call_t& call)
    {
        for (size_t i = 0; i < 6; i++) {
            char c = shifted[i] >> 1;
            char ref = (i < call.size() && call[i] != '\0') ? call[i] : ' ';
            if (c != ref) return false;
        }
        return true;
    }

    /**
     * Find the offset of the info field (control + PID + payload) in linear_buf_.
     * Returns the byte offset, or 0 if not found (frame too short).
     *
     * AX.25 frame layout:
     *   dest(7) + src(7) + digi(0-56) + ctrl(1) + pid(1) + info(N)
     * The last address byte has C-bit (bit 0) set.
     */
    size_t find_info_offset() const
    {
        size_t pos = 14; // skip dest(7) + src(7)
        size_t addr_count = 0;
        while (pos + 7 <= linear_len_ && addr_count < 8) {
            addr_count++;
            if (linear_buf_[pos + 6] & 0x01) break; // C-bit = last address
            pos += 7;
        }
        pos += 7; // skip past the last address (including the one that had C-bit)
        return pos; // now at control byte
    }

    /**
     * Returns true if the frame should be considered for digipeating.
     * Buffer bytes are AX.25 shifted (left by 1).
     */
    bool is_aprs_frame()
    {
        if (linear_len_ < 7) return false;

        // Right-shift the 6-byte destination callsign
        char dest[7];
        for (int i = 0; i < 6; i++) dest[i] = linear_buf_[i] >> 1;
        dest[6] = '\0';

        // APRS TOCALLs start with "AP"
        if (dest[0] == 'A' && dest[1] == 'P') return true;
        // Other digipeated TOCALLs
        if (std::strncmp(dest, "ALL", 3) == 0) return true;
        if (std::strncmp(dest, "BEACON", 6) == 0) return true;
        if (std::strncmp(dest, "CQ", 2) == 0) return true;
        if (std::strncmp(dest, "QST", 3) == 0) return true;
        if (std::strncmp(dest, "GPS", 3) == 0) return true;
        // Also digipeat frames addressed to our aliases
        for (size_t i = 0; i < kiss::NUMBER_OF_ALIASES; i++) {
            auto& a = cfg_.aliases[i];
            if (!a.set || !a.use) continue;
            if (match_shifted_callsign(linear_buf_.data(), a.call)) return true;
        }
        return false;
    }

    /**
     * Returns true if the alias callsign matches a known n-N prefix pattern
     * (WIDE, TRACE, RELAY, ECHO, GATE, TEMP). n-N routing uses SSID as hop
     * count; explicit routing does not.
     */
    static bool is_nN_alias(const kiss::call_t& call)
    {
        static const char* nN_prefixes[] = {"WIDE", "TRACE", "RELAY", "ECHO", "GATE", "TEMP"};
        for (auto prefix : nN_prefixes) {
            size_t plen = std::strlen(prefix);
            bool match = true;
            for (size_t i = 0; i < plen; i++) {
                if (call[i] == '\0' || call[i] != prefix[i]) { match = false; break; }
            }
            if (match) return true;
        }
        return false;
    }

    /**
     * Encode our callsign (mycall) as a 7-byte AX.25 shifted address block.
     * The SSID byte is set with H-bit (bit 7) to mark "has been repeated".
     *
     * mycall is a call_t (char[8]), NUL-padded. We compare 6 chars, pad with spaces.
     * SSID is taken from the mycall field if present, or 0.
     *
     * out must point to a 7-byte buffer.
     */
    void encode_mycall_address(uint8_t* out, uint8_t ssid, bool set_h_bit)
    {
        auto& mycall = cfg_.mycall;
        for (size_t i = 0; i < 6; i++) {
            char c = (i < mycall.size() && mycall[i] != '\0') ? mycall[i] : ' ';
            out[i] = static_cast<uint8_t>(c << 1);
        }
        // SSID byte: (ssid << 1) | H-bit (0x80 if set) | reserved bits
        out[6] = (ssid << 1) | (set_h_bit ? 0x80 : 0x00);
    }

    // ========================================================================
    // Routing: can_repeat
    // ========================================================================

    /**
     * Can the frame be digipeated?  Iterator version -- accepts any forward
     * iterator yielding uint8_t-compatible values.
     *
     * Rules:
     * 1. Must be a UI frame
     * 2. Must be an APRS frame (dest starts with AP, ALL, BEACON, CQ, QST, or GPS)
     * 3. Must not be addressed to us directly (prevent loops)
     * 4. Must not be in dedupe history
     * 5. Must match an active alias via AX.25 address field comparison
     *    (first-unmatched).  When ROUTING_PREEMPT_FRONT is set, falls back
     *    to a path scan for our own callsign (mycall) if the normal alias
     *    scan fails.  In both cases, dedupe and APRS-frame checks apply.
     */
    template <typename InputIt>
    const kiss::Alias* can_repeat(InputIt first, InputIt last)
    {
        // Reset match state on every call.  Single-threaded digipeater task.
        match_type_ = MatchType::None;
        match_addr_offset_ = 0;
        match_is_nN_ = false;
        match_alias_ = nullptr;

        if (!cfg_.digipeater_enabled) return nullptr;

        // Copy input to linear buffer
        linear_len_ = 0;
        for (auto it = first; it != last && linear_len_ < LINEAR_BUF_SIZE; ++it)
            linear_buf_[linear_len_++] = static_cast<uint8_t>(*it);

        if (linear_len_ < 14) return nullptr;

        if (!is_aprs_frame()) return nullptr;

        // Don't digipeat frames addressed to us.
        if (match_shifted_callsign(linear_buf_.data(), cfg_.mycall)) return nullptr;

        // Don't digipeat our own frames (source == mycall).
        if (match_shifted_callsign(linear_buf_.data() + 7, cfg_.mycall)) return nullptr;

        // Don't digipeat if our callsign already appears in the path.
        //   - If H-bit set: already processed, reject.
        //   - If no H-bit but not at the first unmatched position: would
        //     create a duplicate, reject -- UNLESS ROUTING_PREEMPT_FRONT is
        //     set, in which case preempt_front allows the match and the
        //     rewrite will truncate the path at our position.
        bool preempt_enabled =
            (cfg_.routing_mode & kiss::hardware::ROUTING_PREEMPT_FRONT) != 0;
        size_t path_end = 14;
        while (path_end + 7 <= linear_len_) {
            if (match_shifted_callsign(linear_buf_.data() + path_end, cfg_.mycall)) {
                bool repeated = (linear_buf_[path_end + 6] & 0x80) != 0;
                if (repeated) return nullptr;
                if (!preempt_enabled) {
                    size_t first_unmatched = 14;
                    while (first_unmatched <= path_end) {
                        if (!(linear_buf_[first_unmatched + 6] & 0x80)) break;
                        first_unmatched += 7;
                    }
                    if (first_unmatched != path_end) return nullptr;
                }
            }
            if (linear_buf_[path_end + 6] & 0x01) break;
            path_end += 7;
        }

        // Check dedupe
        uint32_t crc = compute_dedupe_key(linear_buf_.data(), linear_len_);
        if (is_duplicate(crc)) return nullptr;

        // Scan for first unmatched address matching an alias.
        size_t pos = 14;
        while (pos + 7 <= linear_len_) {
            bool repeated = (linear_buf_[pos + 6] & 0x80) != 0;
            if (!repeated) {
                uint8_t ssid = (linear_buf_[pos + 6] >> 1) & 0x0F;

                for (size_t i = 0; i < kiss::NUMBER_OF_ALIASES; i++) {
                    auto& a = cfg_.aliases[i];
                    if (!a.set || !a.use || a.hops == 0) continue;

                    if (!match_shifted_callsign(linear_buf_.data() + pos, a.call))
                        continue;

                    bool is_nN = is_nN_alias(a.call);
                    if (is_nN) {
                        if (ssid > 0 && ssid <= a.hops) {
                            match_type_ = MatchType::Alias;
                            match_addr_offset_ = pos;
                            match_is_nN_ = true;
                            match_alias_ = &a;
                            record_frame(crc);
                            return match_alias_;
                        }
                    } else {
                        match_type_ = MatchType::Alias;
                        match_addr_offset_ = pos;
                        match_is_nN_ = false;
                        match_alias_ = &a;
                        record_frame(crc);
                        return match_alias_;
                    }
                }
            }
            if (linear_buf_[pos + 6] & 0x01) break;
            pos += 7;
        }

        // Preempt-front fallback: if no alias matched, scan the entire
        // path for our own callsign (mycall).  This implements the standard
        // APRS "I see myself in the path, I'll take it from here" rule:
        // route the frame, mark mycall with H-bit, and drop everything
        // after.  Only fires when ROUTING_PREEMPT_FRONT is set in
        // routing_mode.  The "mycall with H-bit" guard above already
        // prevents re-processing an already-handled frame.
        if ((cfg_.routing_mode & kiss::hardware::ROUTING_PREEMPT_FRONT) != 0) {
            size_t scan_pos = 14;
            while (scan_pos + 7 <= linear_len_) {
                if (match_shifted_callsign(linear_buf_.data() + scan_pos, cfg_.mycall)) {
                    // H-bit guard above already filtered these out, but
                    // skip defensively.
                    bool repeated = (linear_buf_[scan_pos + 6] & 0x80) != 0;
                    if (!repeated) {
                        match_type_ = MatchType::PreemptFront;
                        match_addr_offset_ = scan_pos;
                        match_is_nN_ = false;
                        match_alias_ = nullptr;
                        record_frame(crc);
                        // Sentinel: returns non-null with match_alias_ = nullptr.
                        // Callers should always check match_type_ first.
                        return reinterpret_cast<const kiss::Alias*>(this);
                    }
                }
                if (linear_buf_[scan_pos + 6] & 0x01) break;
                scan_pos += 7;
            }
        }

        return nullptr;
    }

    /**
     * Convenience overload for raw pointer + length.
     */
    const kiss::Alias* can_repeat(const uint8_t* data, size_t len)
    {
        return can_repeat(data, data + len);
    }

    // ========================================================================
    // Routing: rewrite_frame
    // ========================================================================

    /**
     * Core routing implementation.  Operates on linear_buf_ (already populated
     * by can_repeat() or the iterator-based rewrite_frame).  Writes the
     * rewritten frame to `out` and sets `out_len`.
     *
     * Returns true if routing was applied, false if declined.
     *
     * The match location is taken from the state populated by can_repeat()
     * (match_type_, match_addr_offset_, match_is_nN_, match_alias_).  When
     * match_type_ is PreemptFront the path is truncated at our callsign
     * position; otherwise the standard alias decrement / substitution logic
     * runs.
     */
    bool rewrite_frame_impl(uint8_t* out, size_t& out_len, size_t out_capacity)
    {
        if (linear_len_ < 14) return false;

        uint8_t routing_mode = cfg_.routing_mode;
        bool substitute = (routing_mode & kiss::hardware::ROUTING_SUBSTITUTE) != 0;
        bool skip_complete = (routing_mode & kiss::hardware::ROUTING_SKIP_COMPLETE) != 0;

        // Determine match state.  Prefer the state captured by can_repeat();
        // if absent (e.g. caller invoked rewrite_frame() without first
        // calling can_repeat()), fall back to a scan.  In normal digipeater
        // flow can_repeat() always runs first.
        bool is_preempt = (match_type_ == MatchType::PreemptFront);
        bool is_alias_match = (match_type_ == MatchType::Alias);
        size_t match_addr_offset = match_addr_offset_;
        bool match_is_nN = match_is_nN_;

        if (match_type_ == MatchType::None) {
            // Fallback scan for callers that bypass can_repeat().  This
            // matches the existing behavior so existing tests keep working
            // when rewrite_frame() is called standalone.
            match_addr_offset = 0;
            match_is_nN = false;
            bool found = false;
            size_t pos = 14;
            while (pos + 7 <= linear_len_) {
                bool repeated = (linear_buf_[pos + 6] & 0x80) != 0;
                if (!repeated) {
                    uint8_t ssid = (linear_buf_[pos + 6] >> 1) & 0x0F;
                    for (size_t i = 0; i < kiss::NUMBER_OF_ALIASES; i++) {
                        auto& a = cfg_.aliases[i];
                        if (!a.set || !a.use || a.hops == 0) continue;
                        if (!match_shifted_callsign(linear_buf_.data() + pos, a.call))
                            continue;
                        bool is_nN = is_nN_alias(a.call);
                        if (is_nN) {
                            if (ssid > 0 && ssid <= a.hops) {
                                match_addr_offset = pos;
                                match_is_nN = true;
                                found = true;
                                break;
                            }
                        } else {
                            match_addr_offset = pos;
                            match_is_nN = false;
                            found = true;
                            break;
                        }
                    }
                    if (found) break;
                }
                if (linear_buf_[pos + 6] & 0x01) break;
                pos += 7;
            }
            if (!found) return false;
            is_alias_match = true;
            // is_preempt stays false in fallback path.
        }

        // Find the info field offset (control + PID + payload start)
        size_t info_offset = find_info_offset();
        if (info_offset >= linear_len_) return false;

        // Build the new frame in the output buffer.
        // Layout: dest(7) + src(7) + [new digi path] + ctrl + pid + info
        out_len = 0;

        // 1. Copy dest(7) + src(7) byte-for-byte (never change)
        if (out_capacity < 14) return false;
        std::memcpy(out, linear_buf_.data(), 14);
        out_len = 14;

        // Parse original path into array entries.
        constexpr size_t ADDR_SIZE = 7;
        std::array<std::array<uint8_t, ADDR_SIZE>, MAX_PATH_ADDRS> path_addrs{};
        size_t path_count = 0;

        size_t orig_pos = 14;
        while (orig_pos + ADDR_SIZE <= linear_len_ && path_count < MAX_PATH_ADDRS) {
            std::memcpy(path_addrs[path_count].data(), linear_buf_.data() + orig_pos, ADDR_SIZE);
            path_count++;
            if (linear_buf_[orig_pos + 6] & 0x01) break;
            orig_pos += ADDR_SIZE;
        }

        // match_idx is the index of the matched address in the path_addrs
        // array.
        size_t match_idx = (match_addr_offset - 14) / ADDR_SIZE;

        // Apply skip_complete: filter out completed addresses before the match.
        std::array<std::array<uint8_t, ADDR_SIZE>, MAX_PATH_ADDRS + 1> new_path{};
        size_t new_path_count = 0;

        // All addresses are preserved.  preempt_front only sets H-bit on
        // our entry; it does NOT truncate the path (that's preempt_truncate).
        for (size_t i = 0; i < path_count; i++) {
            if (skip_complete && i < match_idx) {
                uint8_t ssid_byte = path_addrs[i][6];
                uint8_t ssid = (ssid_byte >> 1) & 0x0F;
                bool h_bit = (ssid_byte & 0x80) != 0;
                if (ssid == 0 && h_bit) continue;
            }
            new_path[new_path_count] = path_addrs[i];
            new_path_count++;
        }

        // Recalculate match_idx after skip_complete filtering.
        size_t skipped = 0;
        for (size_t i = 0; i < match_idx; i++) {
            if (skip_complete) {
                uint8_t ssid_byte = path_addrs[i][6];
                uint8_t ssid = (ssid_byte >> 1) & 0x0F;
                bool h_bit = (ssid_byte & 0x80) != 0;
                if (ssid == 0 && h_bit) skipped++;
            }
        }
        size_t new_match_idx = match_idx - skipped;

        // 3. Handle the matched address based on routing type.
        uint8_t& matched_ssid_byte = new_path[new_match_idx][6];
        uint8_t current_ssid = (matched_ssid_byte >> 1) & 0x0F;

        if (is_preempt) {
            // Preempt_front: set H-bit on our callsign.  Do not decrement
            // SSID and do not insert another callsign entry -- the path
            // already contains us, we just mark ourselves and stop here.
            matched_ssid_byte |= 0x80;
            // Preserve C-bit if it was on our entry.
            // (The C-bit clear-pass below also handles it.)
        } else if (match_is_nN) {
            // n-N routing: decrement hop count
            uint8_t new_ssid = current_ssid - 1;
            matched_ssid_byte = (matched_ssid_byte & 0x01) | (new_ssid << 1);
        } else {
            // Explicit routing: no decrement, just mark as repeated (H-bit)
            matched_ssid_byte |= 0x80;
        }

        // 4. Insert or substitute our callsign with H-bit set.
        // Skip this step entirely for preempt_front: we already marked
        // ourselves in place; inserting another DIGI* would duplicate.
        if (!is_preempt) {
            // n-N routing always inserts. Explicit routing inserts unless
            // the matched address is our own callsign (would be a duplicate).
            bool do_substitute = substitute && (match_is_nN && current_ssid == 1);
            bool matched_is_mycall = match_shifted_callsign(
                new_path[new_match_idx].data(), cfg_.mycall);

            if (match_is_nN || !matched_is_mycall) {
                if (do_substitute) {
                    encode_mycall_address(new_path[new_match_idx].data(), 0, match_is_nN);
                    uint8_t c_bit = path_addrs[match_idx][6] & 0x01;
                    new_path[new_match_idx][6] |= c_bit;
                } else {
                    if (new_path_count < MAX_PATH_ADDRS) {
                        for (size_t i = new_path_count; i > new_match_idx; i--) {
                            new_path[i] = new_path[i - 1];
                        }
                        encode_mycall_address(new_path[new_match_idx].data(), 0, match_is_nN);
                        new_path_count++;
                    }
                }
            }
        }

        // 5. Copy the new path to the output buffer.
        for (size_t i = 0; i < new_path_count && out_len + ADDR_SIZE <= out_capacity; i++) {
            std::memcpy(out + out_len, new_path[i].data(), ADDR_SIZE);
            out_len += ADDR_SIZE;
        }

        // 6. Set C-bit on the last address, clear on all others.
        if (out_len > 14) {
            out[out_len - 1] |= 0x01;
        }
        for (size_t i = 14; i + ADDR_SIZE < out_len; i += ADDR_SIZE) {
            out[i + 6] &= ~0x01;
        }

        // 7. Copy info field.
        for (size_t i = info_offset; i < linear_len_ && out_len < out_capacity; i++) {
            out[out_len++] = linear_buf_[i];
        }

        // Suppress unused warning when alias match is found without
        // preempt_front ever firing.
        (void)is_alias_match;

        return out_len > 14;
    }

    /**
     * Rewrite the frame for digipeating.  Iterator version -- accepts any
     * forward iterator yielding uint8_t-compatible values.
     *
     * The caller must have called can_repeat() first (to record the frame
     * in the dedupe ring).
     *
     * Returns true if routing was applied, false if declined or buffer too small.
     */
    template <typename InputIt>
    bool rewrite_frame(InputIt first, InputIt last,
                       uint8_t* out, size_t& out_len, size_t out_capacity)
    {
        // Copy input to linear buffer
        linear_len_ = 0;
        for (auto it = first; it != last && linear_len_ < LINEAR_BUF_SIZE; ++it)
            linear_buf_[linear_len_++] = static_cast<uint8_t>(*it);

        return rewrite_frame_impl(out, out_len, out_capacity);
    }

    /**
     * Convenience overload for raw pointer + length.
     */
    bool rewrite_frame(const uint8_t* data, size_t len,
                       uint8_t* out, size_t& out_len, size_t out_capacity)
    {
        return rewrite_frame(data, data + len, out, out_len, out_capacity);
    }
};

}} // mobilinkd::tnc

#endif // MOBILINKD__TNC__DIGIPEATERCORE_HPP_
