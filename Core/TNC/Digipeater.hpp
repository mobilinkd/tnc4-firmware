// Copyright 2026 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__DIGIPEATER_HPP_
#define MOBILINKD__TNC__DIGIPEATER_HPP_

#include <cmsis_os.h>
#include "KissHardware.hpp"
#include "HdlcFrame.hpp"
#include "aprsroute.hpp"

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
 * APRS digipeater with deduplication and libaprsroute-based routing.
 *
 * Memory model: segmented IoFrame -> linear buffer -> libaprsroute -> new IoFrame
 * - 330-byte linear buffer (10 addresses x 7 + 2 protocol + 256 info + 2 CRC)
 * - 32-entry dedupe ring (CRC32 of source + info)
 * - 30-second dedupe window
 */
struct Digipeater
{
    static constexpr size_t LINEAR_BUF_SIZE = 330;
    static constexpr size_t MAX_DEDUPE_ENTRIES = 32;
    static constexpr size_t MAX_PATH_ADDRS = 8;
    static constexpr size_t ADDR_STR_LEN = 10;

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

    // libaprsroute reusable state (zero-heap)
    aprs::router::route_state route_state_;
    aprs::router::routing_state routing_state_;

    Digipeater(const kiss::Alias* aliases, const kiss::Beacon* beacons)
    : aliases_(aliases), beacons_(beacons)
    {
        dedupe_ring_.fill({0, 0, false});
        init_router_state();
    }

    void init_router_state()
    {
        auto& hw = kiss::settings();
        std::string_view router_addr(hw.mycall.data(), strnlen(hw.mycall.data(), hw.mycall.size()));

        // Build explicit address list from aliases with use=true
        std::array<std::string_view, MAX_PATH_ADDRS> expl_addrs;
        size_t expl_count = 0;
        // Build n-N address list from aliases with use=true and set=true
        std::array<std::string_view, MAX_PATH_ADDRS> nN_addrs;
        size_t nN_count = 0;

        for (size_t i = 0; i < kiss::NUMBER_OF_ALIASES; i++) {
            auto& a = aliases_[i];
            if (!a.set || !a.use) continue;
            std::string_view alias_str(a.call.data(), strnlen(a.call.data(), a.call.size()));

            // Detect if this is an n-N style address (has digits) or explicit
            bool is_nN = false;
            for (auto c : a.call) {
                if (c >= '0' && c <= '9') { is_nN = true; break; }
                if (c == '\0') break;
            }
            // Also check for known n-N prefixes
            if (alias_str.substr(0, 4) == "WIDE" || alias_str.substr(0, 5) == "TRACE" ||
                alias_str.substr(0, 5) == "RELAY" || alias_str.substr(0, 4) == "ECHO" ||
                alias_str.substr(0, 4) == "GATE") {
                is_nN = true;
            }

            if (is_nN && expl_count < MAX_PATH_ADDRS) {
                nN_addrs[nN_count++] = alias_str;
            } else if (expl_count < MAX_PATH_ADDRS) {
                expl_addrs[expl_count++] = alias_str;
            }
        }

        aprs::router::routing_option opts = aprs::router::routing_option::none;
        if (hw.routing_mode & kiss::hardware::ROUTING_PREEMPT_FRONT)
            opts = opts | aprs::router::routing_option::preempt_front;
        if (hw.routing_mode & kiss::hardware::ROUTING_PREEMPT_TRUNCATE)
            opts = opts | aprs::router::routing_option::preempt_truncate;
        if (hw.routing_mode & kiss::hardware::ROUTING_PREEMPT_DROP)
            opts = opts | aprs::router::routing_option::preempt_drop;
        if (hw.routing_mode & kiss::hardware::ROUTING_PREEMPT_MARK)
            opts = opts | aprs::router::routing_option::preempt_mark;
        if (hw.routing_mode & kiss::hardware::ROUTING_SUBSTITUTE)
            opts = opts | aprs::router::routing_option::substitute_complete_n_N_address;
        if (hw.routing_mode & kiss::hardware::ROUTING_SKIP_COMPLETE)
            opts = opts | aprs::router::routing_option::skip_complete_n_N_address;

        aprs::router::init_router(
            router_addr,
            expl_addrs.begin(), expl_addrs.begin() + expl_count,
            nN_addrs.begin(), nN_addrs.begin() + nN_count,
            opts, false, route_state_);
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
            if (buf[pos + 6] & 0x80) break; // last address
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
      * call_t is NUL-padded — compare up to the NUL or first 6 chars.
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
     * Parse AX.25 frame from linear buffer into from/to/path components.
     * AX.25: dest(7) + src(7) + digi(0-56) + ctrl(1) + pid(1) + info(N)
     */
    bool parse_ax25_frame(
        std::string_view& from,
        std::string_view& to,
        std::array<std::string_view, MAX_PATH_ADDRS>& path,
        size_t& path_len)
    {
        if (linear_len_ < 14) return false;

        // Source callsign: bytes 7-13 (6 char + SSID)
        // Destination: bytes 0-6
        char src_buf[10] = {};
        char dst_buf[10] = {};

        // Decode AX.25 address (shifted left by 1, strip HDLC bit)
        for (int i = 0; i < 6; i++) {
            src_buf[i] = linear_buf_[7 + i] >> 1;
            dst_buf[i] = linear_buf_[i] >> 1;
        }
        // Remove trailing spaces from callsign
        for (int i = 5; i >= 0; i--) {
            if (src_buf[i] == ' ') src_buf[i] = '\0';
            else break;
        }
        for (int i = 5; i >= 0; i--) {
            if (dst_buf[i] == ' ') dst_buf[i] = '\0';
            else break;
        }
        // SSID
        uint8_t src_ssid = (linear_buf_[13] >> 1) & 0x0F;
        uint8_t dst_ssid = (linear_buf_[6] >> 1) & 0x0F;

        // Build strings: callsign-SSID or just callsign
        char from_buf[10] = {};
        char to_buf[10] = {};
        if (src_ssid) snprintf(from_buf, sizeof(from_buf), "%s-%d", src_buf, src_ssid);
        else snprintf(from_buf, sizeof(from_buf), "%s", src_buf);
        if (dst_ssid) snprintf(to_buf, sizeof(to_buf), "%s-%d", dst_buf, dst_ssid);
        else snprintf(to_buf, sizeof(to_buf), "%s", dst_buf);

        // Store in linear buffer as temp strings for string_view usage
        // We use the linear buffer since we're done with the raw frame data
        char* str_area = reinterpret_cast<char*>(linear_buf_.data());
        size_t str_offset = 0;

        snprintf(str_area + str_offset, 10, "%s", from_buf);
        str_offset += 10;
        snprintf(str_area + str_offset, 10, "%s", to_buf);
        str_offset += 10;

        from = std::string_view(str_area, strnlen(str_area, 9));
        to = std::string_view(str_area + 10, strnlen(str_area + 10, 9));

        // Parse digipeater path: 7 bytes each, starting at byte 14
        size_t pos = 14;
        path_len = 0;
        while (pos + 7 <= linear_len_ && path_len < MAX_PATH_ADDRS) {
            char addr_buf[10] = {};
            for (int i = 0; i < 6; i++) {
                addr_buf[i] = linear_buf_[pos + i] >> 1;
            }
            // Remove trailing spaces
            for (int i = 5; i >= 0; i--) {
                if (addr_buf[i] == ' ') addr_buf[i] = '\0';
                else break;
            }
            uint8_t ssid = (linear_buf_[pos + 6] >> 1) & 0x0F;
            bool has_been_set = (linear_buf_[pos + 6] & 0x80) != 0;

            // Store in linear buffer
            if (has_been_set) {
                if (ssid) snprintf(str_area + str_offset, 10, "%s-%d*", addr_buf, ssid);
                else snprintf(str_area + str_offset, 10, "%s*", addr_buf);
            } else {
                if (ssid) snprintf(str_area + str_offset, 10, "%s-%d", addr_buf, ssid);
                else snprintf(str_area + str_offset, 10, "%s", addr_buf);
            }
            path[path_len] = std::string_view(str_area + str_offset, strnlen(str_area + str_offset, 9));
            str_offset += 10;
            path_len++;

            if (linear_buf_[pos + 6] & 0x80) break; // last address
            pos += 7;
        }

        return true;
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
     * Rewrite the frame using libaprsroute.
     *
     * Copies the frame to a linear buffer, runs it through the router,
     * builds a new IoFrame, and returns it.
     */
    hdlc::IoFrame* rewrite_frame(hdlc::IoFrame* frame)
    {
        // Parse into string_views for libaprsroute
        copy_to_linear(frame);
        if (linear_len_ < 14) return frame;

        std::string_view from, to;
        std::array<std::string_view, MAX_PATH_ADDRS> path;
        size_t path_len = 0;

        if (!parse_ax25_frame(from, to, path, path_len)) return frame;

        // Build the routed path output arrays
        std::array<std::array<char, ADDR_STR_LEN>, MAX_PATH_ADDRS> routed_path{};
        std::array<size_t, MAX_PATH_ADDRS> routed_path_sizes{};

        // Re-init router state with current settings (may have changed)
        init_router_state();

        // Route the packet using the post-init overload (no heap)
        auto [routed_end, sizes_end, actions_end, success] =
            aprs::router::try_route_packet(
                from, to,
                path.begin(), path.begin() + path_len,
                routed_path.begin(),
                routed_path_sizes.begin(),
                aprs::router::detail::discard_output_iterator{},
                routing_state_, route_state_);

        if (!success || routing_state_ != aprs::router::routing_state::routed) {
            return frame; // Router declined -- return original (will be released by caller)
        }

        // Count routed path entries
        size_t routed_count = 0;
        for (auto it = routed_path.begin(); it != routed_end; ++it) {
            routed_count++;
        }

        if (routed_count == 0) return frame;

        // Build a new IoFrame with the routed path
        auto new_frame = hdlc::acquire();
        if (new_frame == nullptr) {
            ERROR("Digipeater: OOM acquiring new frame");
            return frame;
        }

        // Build the re-written frame: dest(7) + src(7) + digi_path + ctrl + pid + info
        // Re-encode destination from `to` string_view back to AX.25 shifted format
        // Re-encode source from `from` string_view
        // Re-encode digipeater path from libaprsroute output
        // Copy info field from original

        // Helper to encode a callsign string to 7-byte shifted AX.25 format
        auto encode_addr = [](std::string_view addr_str, uint8_t* out) {
            // Parse callsign-SSID or callsign*
            std::string_view call;
            int ssid = 0;
            bool has_been_set = false;

            auto star_pos = addr_str.find('*');
            if (star_pos != std::string_view::npos) {
                has_been_set = true;
                addr_str = addr_str.substr(0, star_pos);
            }

            auto dash_pos = addr_str.find('-');
            if (dash_pos != std::string_view::npos && dash_pos < 7) {
                call = addr_str.substr(0, dash_pos);
                auto ssid_str = addr_str.substr(dash_pos + 1);
                if (!ssid_str.empty()) ssid = atoi(ssid_str.data());
            } else {
                call = addr_str;
            }

            // Pad with spaces to 6 chars, shift left by 1
            for (int i = 0; i < 6; i++) {
                if (i < (int)call.size()) out[i] = call[i] << 1;
                else out[i] = ' ' << 1;
            }
            // SSID byte: flags(3) + ssid(4) + 0
            out[6] = (ssid << 1) | (has_been_set ? 0x80 : 0x00);
        };

        // Build new frame in linear buffer
        std::array<uint8_t, LINEAR_BUF_SIZE> new_buf{};
        size_t new_len = 0;

        // Destination (7 bytes)
        encode_addr(to, new_buf.data() + new_len);
        new_len += 7;

        // Source (7 bytes)
        encode_addr(from, new_buf.data() + new_len);
        new_len += 7;

        // Digipeater path from libaprsroute
        size_t path_idx = 0;
        for (auto it = routed_path.begin(); it != routed_end && path_idx < MAX_PATH_ADDRS; ++it, ++path_idx) {
            std::string_view addr((*it).data(), routed_path_sizes[path_idx]);
            encode_addr(addr, new_buf.data() + new_len);
            new_len += 7;
        }
        // Mark the last address as "last in chain" (bit 0 set)
        if (new_len >= 7) {
            new_buf[new_len - 1] |= 0x01;
        }

        // Copy info field: control + pid + info from original
        // Find where info starts in the original
        size_t orig_pos = 14;
        size_t addr_count = 0;
        while (orig_pos + 7 <= linear_len_ && addr_count < 8) {
            addr_count++;
            if (linear_buf_[orig_pos + 6] & 0x80) break;
            orig_pos += 7;
        }
        orig_pos += 7; // skip last addr
        size_t info_len = (orig_pos + 2 <= linear_len_) ? linear_len_ - orig_pos : 0;
        if (info_len > 0 && orig_pos + 2 <= linear_len_) {
            // Copy control + pid + info
            for (size_t i = orig_pos; i < linear_len_ && new_len < LINEAR_BUF_SIZE; i++) {
                new_buf[new_len++] = linear_buf_[i];
            }
        }

        // Fill the new IoFrame with the assembled packet
        for (size_t i = 0; i < new_len; i++) {
            if (!new_frame->push_back(new_buf[i])) {
                ERROR("Digipeater: OOM pushing to new frame");
                hdlc::release(new_frame);
                return frame;
            }
        }

        // Add FCS
        new_frame->add_fcs();

        return new_frame;
    }
};

}} // mobilinkd::tnc

#endif // __cplusplus

#endif // MOBILINKD__TNC__DIGIPEATER_HPP_
