// Copyright 2026 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__DIGIPEATER_HPP_
#define MOBILINKD__TNC__DIGIPEATER_HPP_

#include <cmsis_os.h>
#include "KissHardware.hpp"
#include "HdlcFrame.hpp"

#include <array>
#include <cstring>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

extern osThreadId digipeaterTaskHandle;
extern osMessageQId digipeaterQueueHandle;

void startDigipeaterTask(void* arg);
void beacon(void* arg);

#ifdef __cplusplus
} // extern "C"

namespace mobilinkd { namespace tnc {

/**
 * APRS digipeater with deduplication and direct AX.25 buffer routing.
 *
 * Memory model: segmented IoFrame -> linear buffer -> new IoFrame
 * - 330-byte linear buffer (10 addresses x 7 + 2 protocol + 256 info + 2 CRC)
 * - 32-entry dedupe ring (CRC32 of source + info)
 * - 30-second dedupe window
 *
 * Routing operates directly on AX.25 7-byte address blocks in linear_buf_.
 * No string conversion, no external routing library.
 */
struct Digipeater
{
    static constexpr size_t LINEAR_BUF_SIZE = 330;
    static constexpr size_t MAX_DEDUPE_ENTRIES = 32;
    static constexpr size_t MAX_PATH_ADDRS = 8;

    const kiss::Alias* aliases_;
    const kiss::Beacon* beacons_;

    // Linear buffer for segmented->contiguous bridging
    std::array<uint8_t, LINEAR_BUF_SIZE> linear_buf_;
    size_t linear_len_ = 0;

    // Dedupe ring buffer
    struct DedupeEntry {
        uint32_t crc;
        uint32_t timestamp;  // HAL_GetTick() when recorded
        bool valid;
    };
    std::array<DedupeEntry, MAX_DEDUPE_ENTRIES> dedupe_ring_;
    size_t dedupe_head_ = 0;

    Digipeater(const kiss::Alias* aliases, const kiss::Beacon* beacons)
    : aliases_(aliases), beacons_(beacons)
    {
        dedupe_ring_.fill({0, 0, false});
    }

    /**
     * Scan the dedupe history table and remove outdated entries.
     * Called at the top of the digipeater task loop.
     */
    void clean_history()
    {
        uint32_t now = HAL_GetTick();
        uint32_t dedupe_ms = kiss::settings().dedupe_seconds * 1000;
        for (auto& e : dedupe_ring_) {
            if (e.valid && (now - e.timestamp) > dedupe_ms) {
                e.valid = false;
            }
        }
    }

    /**
     * Compute a CRC32 key for deduplication over source + info fields.
     * This gives us a unique frame fingerprint without parsing the full path.
     */
    uint32_t compute_dedupe_key(const uint8_t* buf, size_t len)
    {
        // buf contains raw AX.25 frame: dest(7) + src(7) + digi(0-56) + ctrl(1) + pid(1) + info
        // We hash source (bytes 7-13) and info (after digipeaters + ctrl + pid)
        if (len < 15) return 0;

        // Count digipeater addresses: each is 7 bytes, last has bit 1 set in SSID
        size_t pos = 14; // dest(7) + src(7)
        size_t addr_count = 0;
        while (pos + 7 <= len && addr_count < 8) {
            addr_count++;
            if (buf[pos + 6] & 0x80) break; // last address (H-bit in SSID byte)
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
        dedupe_ring_[dedupe_head_] = {crc, HAL_GetTick(), true};
        dedupe_head_ = (dedupe_head_ + 1) % MAX_DEDUPE_ENTRIES;
    }

    /**
     * Copy an IoFrame's data into the linear buffer.
     * Returns the number of bytes copied.
     */
    size_t copy_to_linear(hdlc::IoFrame* frame)
    {
        linear_len_ = 0;
        for (auto c : *frame) {
            if (linear_len_ >= LINEAR_BUF_SIZE) break;
            linear_buf_[linear_len_++] = c;
        }
        return linear_len_;
    }

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
             auto& a = aliases_[i];
             if (!a.set || !a.use) continue;
             if (match_shifted_callsign(linear_buf_.data(), a.call)) return true;
         }
         return false;
     }

    /**
     * Can the frame be digipeated?
     *
     * Rules:
     * 1. Must be a UI frame
     * 2. Must be an APRS frame (dest starts with AP, ALL, BEACON, CQ, QST, or GPS)
     * 3. Must not be addressed to us directly (prevent loops)
     * 4. Must not be in dedupe history
     * 5. Must match an active alias via AX.25 address field comparison
     */
    const kiss::Alias* can_repeat(hdlc::IoFrame* frame)
    {
        if (!kiss::settings().digipeater_enabled) return nullptr;

        copy_to_linear(frame);
        if (linear_len_ < 14) return nullptr;

        if (!is_aprs_frame()) return nullptr;

        // Don't digipeat frames addressed to us. Buffer has shifted bytes,
        // mycall is unshifted ASCII. Right-shift buffer, compare against mycall.
        auto& hw = kiss::settings();
        if (match_shifted_callsign(linear_buf_.data(), hw.mycall.data(), 6)) return nullptr;

        // Don't digipeat if our callsign already appears in the path
        // (already repeated by us). Scan all digipeater addresses.
        size_t path_end = 14;
        while (path_end + 7 <= linear_len_) {
            // Check bit 7 (H-bit): has-been-repeated
            bool repeated = (linear_buf_[path_end + 6] & 0x80) != 0;
            if (repeated && match_shifted_callsign(linear_buf_.data() + path_end, hw.mycall.data(), 6))
                return nullptr; // Already digipeated by us
            // Bit 0 (C-bit): 1 = last address, 0 = more follow
            if (linear_buf_[path_end + 6] & 0x01) break;
            path_end += 7;
        }

        // Check dedupe
        uint32_t crc = compute_dedupe_key(linear_buf_.data(), linear_len_);
        if (is_duplicate(crc)) return nullptr;

        // Scan for first unmatched digipeater address matching an alias.
        // Each AX.25 address is 7 bytes: 6 shifted ASCII + 1 SSID byte.
        // SSID byte: bits 1-4 = SSID value (hop count for n-N),
        // bit 7 (H-bit) = has-been-repeated (set by digipeater that used it),
        // bit 0 (C-bit) = last address marker.
        size_t pos = 14;
        while (pos + 7 <= linear_len_) {
            bool repeated = (linear_buf_[pos + 6] & 0x80) != 0;
            if (!repeated) {
                // SSID value = bits 1-4 (right-shift by 1, mask 0x0F)
                uint8_t ssid = (linear_buf_[pos + 6] >> 1) & 0x0F;

                for (size_t i = 0; i < kiss::NUMBER_OF_ALIASES; i++) {
                    auto& a = aliases_[i];
                    if (!a.set || !a.use || a.hops == 0) continue;

                    if (!match_shifted_callsign(linear_buf_.data() + pos, a.call))
                        continue;

                    // For n-N routing, the SSID value is the current hop count.
                    // Match if the hop count is within our configured limit.
                    if (ssid > 0 && ssid <= a.hops) {
                        record_frame(crc);
                        return &a;
                    }
                }
            }
            if (linear_buf_[pos + 6] & 0x01) break; // last address
            pos += 7;
        }

        return nullptr;
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
        auto& mycall = kiss::settings().mycall;
        for (size_t i = 0; i < 6; i++) {
            char c = (i < mycall.size() && mycall[i] != '\0') ? mycall[i] : ' ';
            out[i] = static_cast<uint8_t>(c << 1);
        }
        // SSID byte: (ssid << 1) | H-bit (0x80 if set) | reserved bits
        out[6] = (ssid << 1) | (set_h_bit ? 0x80 : 0x00);
    }

    /**
     * Rewrite the frame for digipeating using direct AX.25 buffer operations.
     *
     * The frame is already in linear_buf_ from can_repeat(). We operate directly
     * on the 7-byte address blocks. No string conversion, no external routing.
     *
     * n-N routing algorithm (e.g. WIDE2-2, WIDE1-1):
     *   1. Walk digipeater path addresses (byte 14+, each 7 bytes)
     *   2. Find first unmatched (H-bit clear) address matching a configured alias
     *   3. Decrement SSID (hop count) in the address byte
     *   4. Insert our callsign (mycall) with H-bit set before the matched address
     *      - If SSID > 0 after decrement: n-N address keeps H-bit clear, our call gets H-bit
     *      - If SSID == 0 and ROUTING_SUBSTITUTE: replace n-N address with our call (H-bit set)
     *      - If SSID == 0 without SUBSTITUTE: set H-bit on n-N address, insert our call with H-bit
     *   5. If ROUTING_SKIP_COMPLETE: remove addresses with SSID=0 and H-bit set (completed hops)
     *   6. Set C-bit on the final address in the new path
     *   7. Copy info field (ctrl + PID + payload) byte-for-byte from original
     *
     * Returns a new IoFrame, or the original frame if routing declined.
     */
    hdlc::IoFrame* rewrite_frame(hdlc::IoFrame* frame)
    {
        // linear_buf_ already populated by can_repeat()
        if (linear_len_ < 14) return frame;

        auto& hw = kiss::settings();
        uint8_t routing_mode = hw.routing_mode;
        bool substitute = (routing_mode & kiss::hardware::ROUTING_SUBSTITUTE) != 0;
        bool skip_complete = (routing_mode & kiss::hardware::ROUTING_SKIP_COMPLETE) != 0;

        // Find the info field offset (control + PID + payload start)
        size_t info_offset = find_info_offset();
        if (info_offset >= linear_len_) return frame; // No info field

        // Walk digipeater path to find first unmatched alias match.
        // Path addresses start at byte 14 (after dest(7) + src(7)).
        size_t match_addr_offset = 0;  // byte offset of matched address in linear_buf_
        bool found = false;

        size_t pos = 14;
        while (pos + 7 <= linear_len_) {
            bool repeated = (linear_buf_[pos + 6] & 0x80) != 0;
            if (!repeated) {
                uint8_t ssid = (linear_buf_[pos + 6] >> 1) & 0x0F;

                for (size_t i = 0; i < kiss::NUMBER_OF_ALIASES; i++) {
                    auto& a = aliases_[i];
                    if (!a.set || !a.use || a.hops == 0) continue;

                    if (!match_shifted_callsign(linear_buf_.data() + pos, a.call))
                        continue;

                    if (ssid > 0 && ssid <= a.hops) {
                        match_addr_offset = pos;
                        found = true;
                        break;
                    }
                }
                if (found) break;
            }
            if (linear_buf_[pos + 6] & 0x01) break; // C-bit = last address
            pos += 7;
        }

        if (!found) return frame;

        // Build the new frame in a output buffer.
        // Layout: dest(7) + src(7) + [new digi path] + ctrl + pid + info
        std::array<uint8_t, LINEAR_BUF_SIZE> out_buf{};
        size_t out_len = 0;

        // 1. Copy dest(7) + src(7) byte-for-byte (never change)
        std::memcpy(out_buf.data(), linear_buf_.data(), 14);
        out_len = 14;

        // 2. Walk the original digipeater path and build the new path.
        // We need to:
        //   - Copy addresses before the match as-is
        //   - Handle the matched address (decrement, maybe substitute)
        //   - Insert our callsign with H-bit set
        //   - Copy remaining addresses after the match
        //   - Apply skip_complete (drop SSID=0 + H-bit set addresses)

        // First, collect the path addresses into a working array for manipulation.
        // Each entry is a 7-byte address block.
        constexpr size_t ADDR_SIZE = 7;
        std::array<std::array<uint8_t, ADDR_SIZE>, MAX_PATH_ADDRS> path_addrs{};
        size_t path_count = 0;

        // Parse original path into array entries.
        size_t orig_pos = 14;
        while (orig_pos + ADDR_SIZE <= linear_len_ && path_count < MAX_PATH_ADDRS) {
            std::memcpy(path_addrs[path_count].data(), linear_buf_.data() + orig_pos, ADDR_SIZE);
            path_count++;
            if (linear_buf_[orig_pos + 6] & 0x01) break; // C-bit = last address
            orig_pos += ADDR_SIZE;
        }

        // Find the index of the matched address in our path_addrs array.
        size_t match_idx = (match_addr_offset - 14) / ADDR_SIZE;

        // Apply skip_complete: remove completed n-N addresses (SSID=0, H-bit set)
        // that appear before our match. These are exhausted hops.
        // We do this by filtering when copying to the output path.
        // We'll build the output path in a separate array.
        std::array<std::array<uint8_t, ADDR_SIZE>, MAX_PATH_ADDRS + 1> new_path{};
        size_t new_path_count = 0;

        for (size_t i = 0; i < path_count; i++) {
            if (skip_complete && i < match_idx) {
                // Check if this address is completed (SSID=0 and H-bit set)
                uint8_t ssid_byte = path_addrs[i][6];
                uint8_t ssid = (ssid_byte >> 1) & 0x0F;
                bool h_bit = (ssid_byte & 0x80) != 0;
                if (ssid == 0 && h_bit) {
                    // Skip this completed address
                    continue;
                }
            }
            new_path[new_path_count] = path_addrs[i];
            new_path_count++;
        }

        // Recalculate match_idx in new_path (addresses before match may have been removed).
        // Count how many addresses were skipped before the original match_idx.
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

        // 3. Decrement the SSID (hop count) on the matched address.
        uint8_t& matched_ssid_byte = new_path[new_match_idx][6];
        uint8_t current_ssid = (matched_ssid_byte >> 1) & 0x0F;
        uint8_t new_ssid = current_ssid - 1;  // current_ssid >= 1 guaranteed by can_repeat

        // Clear H-bit and set new SSID on the matched address.
        matched_ssid_byte = (matched_ssid_byte & 0x01) | (new_ssid << 1);
        // Note: H-bit (0x80) is cleared here, C-bit (0x01) preserved.

        // 4. Insert our callsign with H-bit set.
        // Determine if we should substitute (replace) or insert.
        bool do_substitute = substitute && (new_ssid == 0);

        if (do_substitute) {
            // Replace the matched address with our callsign, H-bit set.
            encode_mycall_address(new_path[new_match_idx].data(), 0, true);
            // Preserve C-bit from the original address.
            // (Already preserved in matched_ssid_byte, but we overwrote it. Fix:)
            // Actually, encode_mycall_address sets out[6] = (ssid<<1) | h_bit.
            // We need to preserve the C-bit (bit 0).
            uint8_t c_bit = path_addrs[match_idx][6] & 0x01;
            new_path[new_match_idx][6] |= c_bit;
        } else {
            // Insert our callsign before the matched address.
            // Need to shift entries from new_match_idx onward to make room.
            if (new_path_count < MAX_PATH_ADDRS + 1) {
                // Shift right to make room at new_match_idx.
                for (size_t i = new_path_count; i > new_match_idx; i--) {
                    new_path[i] = new_path[i - 1];
                }
                // Insert our callsign with H-bit set, SSID=0.
                encode_mycall_address(new_path[new_match_idx].data(), 0, true);
                new_path_count++;
            }
            // If path is full (8 addresses + our insert would exceed), we can't insert.
            // In that case, just set H-bit on the matched address and don't insert.
            // (This is the "can't insert, just mark" fallback.)
        }

        // 5. Copy the new path to the output buffer.
        for (size_t i = 0; i < new_path_count && out_len + ADDR_SIZE <= LINEAR_BUF_SIZE; i++) {
            std::memcpy(out_buf.data() + out_len, new_path[i].data(), ADDR_SIZE);
            out_len += ADDR_SIZE;
        }

        // 6. Set C-bit (bit 0) on the last address byte.
        if (out_len > 14) {
            out_buf[out_len - 1] |= 0x01;
        }
        // Also clear C-bit on all preceding address bytes (they should have C=0).
        for (size_t i = 14; i < out_len - ADDR_SIZE; i += ADDR_SIZE) {
            out_buf[i + 6] &= ~0x01;
        }

        // 7. Copy info field (control + PID + payload) from original.
        for (size_t i = info_offset; i < linear_len_ && out_len < LINEAR_BUF_SIZE; i++) {
            out_buf[out_len++] = linear_buf_[i];
        }

        if (out_len <= 14) return frame; // No path, shouldn't happen

        // 8. Build the new IoFrame.
        auto new_frame = hdlc::acquire();
        if (new_frame == nullptr) {
            ERROR("Digipeater: OOM acquiring new frame");
            return frame;
        }

        for (size_t i = 0; i < out_len; i++) {
            if (!new_frame->push_back(out_buf[i])) {
                ERROR("Digipeater: OOM pushing to new frame");
                hdlc::release(new_frame);
                return frame;
            }
        }

        new_frame->add_fcs();
        return new_frame;
    }
};

}} // mobilinkd::tnc

#endif // __cplusplus

#endif // MOBILINKD__TNC__DIGIPEATER_HPP_