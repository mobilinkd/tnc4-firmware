// Explicit/advanced routing tests for the digipeater.
//
// Hand-written edge-case tests that exercise specific can_repeat / SSID
// handling behaviors not covered by the conformance runner:
//
//   - Explicit alias (non-n-N) match: CALLD
//   - H-bit rejection
//   - C-bit interaction
//   - Mycall as destination / source rejection
//   - Disabled digipeater rejection
//   - SSID-aware mycall matching (WX9O-1 vs WX9O-5)
//   - Path-position loop detection for SSID-aware mycall
//   - SSID encoding in rewritten frame
//
// Stub tests exist for routing modes that are intentionally not implemented
// (substitute_explicit, traceless, preempt_*, reject/trap limits, strict,
// route_self, combined).  They are counted as DISABLED so the test output
// makes the gap visible.

#include "test_digipeater_harness.hpp"

#include <array>
#include <iostream>
#include <string>

using namespace mobilinkd::tnc::kiss;
using namespace test;

static int passed = 0;
static int failed = 0;
static int disabled = 0;

#define TEST(name) \
    void name(); \
    struct name##_runner { name##_runner() { std::cout << "  " << #name << "... "; name(); } }; \
    static name##_runner name##_instance; \
    void name()

#define EXPECT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            failed++; \
            return; \
        } \
    } while(0)

#define EXPECT_TRUE(cond) EXPECT(cond, #cond " is false")
#define EXPECT_FALSE(cond) EXPECT(!(cond), #cond " is true")

// ============================================================================
// can_repeat edge-case tests
// ============================================================================

TEST(can_repeat_exact_callsign_match) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("CALLD", 1);
    TestDigipeater digi(cfg);

    // Explicit alias CALLD should match in the path.
    auto buf = parse_ax25_packet("N0CALL>APRS,CALLD-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result != nullptr);
    passed++;
}

TEST(can_repeat_reject_hbit_set) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);

    // WIDE1* means already repeated -- must be skipped.
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE1*:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(can_repeat_match_with_cbit_set) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);

    // WIDE1-1 with C-bit (last in path).  can_repeat spec says we still match.
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result != nullptr);
    passed++;
}

TEST(can_repeat_reject_dest_is_mycall) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);

    auto buf = parse_ax25_packet("N0CALL>DIGI,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(can_repeat_reject_src_is_mycall) {
    // Source field technically can't match a 6-byte SSID-less mycall when
    // mycall is exactly 6 chars; here it's 4 chars so the call shifts in
    // with spaces.  Force a real match by setting mycall to "N0CALL".
    auto cfg = make_test_config("N0CALL");
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);

    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(can_repeat_reject_when_digipeater_disabled) {
    auto cfg = make_test_config("DIGI");
    cfg.digipeater_enabled = 0;  // disabled
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);

    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

// ============================================================================
// SSID-aware mycall tests
// ============================================================================

TEST(ssid_digi_wx9o_1_routes_wx9o_5_source) {
    // Digi is WX9O-1. Frame from WX9O-5 (different SSID = different station).
    // Must be routed -- WX9O-5 is not "our own frame".
    auto cfg = make_test_config("WX9O-1", 0);
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("WX9O-5>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result != nullptr);
    passed++;
}

TEST(ssid_digi_wx9o_1_rejects_wx9o_1_source) {
    // Digi is WX9O-1. Frame from WX9O-1 (same SSID = our own frame).
    // Must be rejected.
    auto cfg = make_test_config("WX9O-1", 0);
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("WX9O-1>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(ssid_digi_wx9o_1_rejects_wx9o_1_dest) {
    // Frame addressed TO WX9O-1 specifically. Must be rejected.
    auto cfg = make_test_config("WX9O-1", 0);
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>WX9O-1,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(ssid_digi_wx9o_1_non_aprs_dest_rejected) {
    // Frame addressed TO WX9O-5 (different SSID from our WX9O-1).
    // Not an APRS TOCALL, so is_aprs_frame() rejects it regardless of SSID.
    auto cfg = make_test_config("WX9O-1", 0);
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>WX9O-5,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(ssid_digi_wx9o_1_path_wx9o_5_not_loop) {
    // WX9O-5 appears in path (different SSID from our WX9O-1).
    // WX9O-5 is a non-empty unused address before the WIDE1-1 match.
    // Per Decision 1 (blocking rule), this packet is dropped -- the
    // reference also blocks here because WX9O-5 is not our callsign
    // and sits between the used prefix (empty) and the alias match.
    auto cfg = make_test_config("WX9O-1", 0);
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,WX9O-5,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(ssid_digi_wx9o_1_path_wx9o_1_first_unmatched) {
    // WX9O-1 at the FIRST unmatched position in path. Current code allows
    // this (someone addressed the frame through us). The alias scan then
    // matches WIDE1-1 and routes.
    // NOTE: this can produce a duplicate mycall insertion -- known issue.
    auto cfg = make_test_config("WX9O-1", 0);
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,WX9O-1,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result != nullptr);
    passed++;
}

TEST(ssid_digi_wx9o_1_path_wx9o_1_not_first_is_loop) {
    // WX9O-1 at a NON-FIRST-UNMATCHED position in path.
    // CALL is unmatched and before us -- this IS a loop, reject.
    auto cfg = make_test_config("WX9O-1", 0);
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,CALL,WX9O-1,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(ssid_encode_mycall_uses_configured_ssid) {
    // When digi inserts itself into the path, it should use SSID=1.
    auto cfg = make_test_config("WX9O-1", 0);
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);
    auto in_buf = parse_ax25_packet("N0CALL>APRS,WIDE1-1:hello");
    auto alias = digi.can_repeat(in_buf.data(), in_buf.size());
    EXPECT_TRUE(alias != nullptr);

    std::array<uint8_t, 330> out_buf{};
    size_t out_len = 0;
    auto ok = digi.rewrite_frame(in_buf.data(), in_buf.size(),
                                 out_buf.data(), out_len, out_buf.size());
    EXPECT_TRUE(ok);
    auto str = ax25_packet_to_string(out_buf.data(), out_len);
    // Should contain WX9O-1* (our callsign with SSID=1 and H-bit)
    EXPECT_TRUE(str.find("WX9O-1*") != std::string::npos);
    passed++;
}

TEST(ssid_zero_still_works) {
    // Backward compat: mycall with no SSID (ssid=0) still matches SSID-0 frames.
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);
    // Source DIGI-0 should be rejected (same as DIGI with ssid=0)
    auto buf = parse_ax25_packet("DIGI>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

// ============================================================================
// Disabled-mode stubs: routing modes not implemented in firmware.
// These exist so the test output shows the gap; no assertion runs.
// ============================================================================

TEST(disabled_substitute_explicit) { disabled++; }
TEST(disabled_traceless_n_N) { disabled++; }
TEST(disabled_reject_limit) { disabled++; }
TEST(disabled_trap_limit) { disabled++; }
TEST(disabled_route_self) { disabled++; }
TEST(disabled_strict) { disabled++; }
TEST(disabled_combined) { disabled++; }

int main() {
    std::cout << "\nDigipeater explicit routing tests\n";
    std::cout << passed << " passed, " << failed << " failed, "
              << disabled << " disabled\n";
    return failed ? 1 : 0;
}
