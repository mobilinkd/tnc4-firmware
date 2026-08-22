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
    cfg.aliases[0] = make_alias("CALLD", 0);  // explicit alias (SSID 0)
    TestDigipeater digi(cfg);

    // Explicit alias CALLD (SSID 0) matches CALLD in the path.
    // NOTE: CALLD-1 (SSID 1) must NOT match -- explicit alias matching is
    // SSID-sensitive (Option A).  Use SSID-0 form here.
    auto buf = parse_ax25_packet("N0CALL>APRS,CALLD:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result != nullptr);
    passed++;
}

TEST(can_repeat_reject_hbit_set) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE", 0x02);
    TestDigipeater digi(cfg);

    // WIDE1* means already repeated -- must be skipped.
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE1*:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(can_repeat_match_with_cbit_set) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE", 0x02);
    TestDigipeater digi(cfg);

    // WIDE1-1 with C-bit (last in path).  can_repeat spec says we still match.
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result != nullptr);
    passed++;
}

TEST(can_repeat_reject_dest_is_mycall) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE", 0x02);
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
    cfg.aliases[0] = make_alias("WIDE", 0x02);
    TestDigipeater digi(cfg);

    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(can_repeat_reject_when_digipeater_disabled) {
    auto cfg = make_test_config("DIGI");
    cfg.digipeater_enabled = 0;  // disabled
    cfg.aliases[0] = make_alias("WIDE", 0x02);
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
    cfg.aliases[0] = make_alias("WIDE", 0x02);
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
    cfg.aliases[0] = make_alias("WIDE", 0x02);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("WX9O-1>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(ssid_digi_wx9o_1_rejects_wx9o_1_dest) {
    // Frame addressed TO WX9O-1 specifically. Must be rejected.
    auto cfg = make_test_config("WX9O-1", 0);
    cfg.aliases[0] = make_alias("WIDE", 0x02);
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
    cfg.aliases[0] = make_alias("WIDE", 0x02);
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
    cfg.aliases[0] = make_alias("WIDE", 0x02);
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
    cfg.aliases[0] = make_alias("WIDE", 0x02);
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
    cfg.aliases[0] = make_alias("WIDE", 0x02);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,CALL,WX9O-1,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(ssid_encode_mycall_uses_configured_ssid) {
    // When digi inserts itself into the path, it should use SSID=1.
    auto cfg = make_test_config("WX9O-1", 0);
    cfg.aliases[0] = make_alias("WIDE", 0x02);
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
    cfg.aliases[0] = make_alias("WIDE", 0x02);
    TestDigipeater digi(cfg);
    // Source DIGI-0 should be rejected (same as DIGI with ssid=0)
    auto buf = parse_ax25_packet("DIGI>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(malformed_empty_slot_dropped) {
    // Empty (placeholder) slot at the first unused position, content
    // after it: the path has a hole -- malformed.  We do not repeat
    // malformed packets and we do not clean them up; we drop them.
    // (libaprsroute routes these and removes the empty slot; deliberate
    // divergence -- conformance case 221.)
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE", 0x02);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(malformed_empty_slot_after_match_routes) {
    // Empty slot AFTER the match (trailing placeholder) is not a hole
    // before the match -- first unused is the match itself, route.
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE", 0x02);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE1-1,:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result != nullptr);
    passed++;
}

TEST(n_N_reject_N_gt_n_never_routed) {
    // FIRM REQUIREMENT (decision 7): N > n is never digipeated.  WIDE2-7
    // asks for 7 remaining hops through class-2 digis -- either network
    // abuse or a corrupted packet.  Serving it would give a broken path
    // up to 7 hops of rebroadcast.  Drop it.
    // (Deliberate divergence from the libaprsroute reference, which
    // serves any N in 1..7 on a configured class.)
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE", 0x04);  // class 2
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE2-7:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(n_N_reject_N_gt_n_wide2_3) {
    // Same rule one step from valid: WIDE2-3 (N=3 > n=2) is not routed.
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE", 0x04);  // class 2
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE2-3:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(n_N_route_N_eq_n_wide2_2) {
    // Sanity: the normal default path still routes.
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE", 0x04);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE2-2:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result != nullptr);
    passed++;
}

TEST(n_N_reject_N_gt_n_blocks_later_match) {
    // An N>n address before a valid match poisons the packet: the
    // blocking rule drops it (WIDE2-7 is a non-empty unused non-mycall
    // address at the first unused position).
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE", 0x04);  // class 2
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE2-7,WIDE2-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result == nullptr);
    passed++;
}

TEST(n_N_class_mask_rejects_unselected_class) {
    // A class-2 alias does NOT serve class 1 or class 3.
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE", 0x04);  // class 2 only
    TestDigipeater digi(cfg);
    auto b1 = parse_ax25_packet("N0CALL>APRS,WIDE1-1:one");
    auto b3 = parse_ax25_packet("N0CALL>APRS,WIDE3-1:three");
    EXPECT_TRUE(digi.can_repeat(b1.data(), b1.size()) == nullptr);
    EXPECT_TRUE(digi.can_repeat(b3.data(), b3.size()) == nullptr);
    passed++;
}

TEST(n_N_class_mask_serves_selected_classes) {
    // Classes 1 and 2 selected: both route, class 3 does not.
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE", 0x06);  // classes 1 and 2
    TestDigipeater digi(cfg);
    auto b1 = parse_ax25_packet("N0CALL>APRS,WIDE1-1:one");
    auto b2 = parse_ax25_packet("N0CALL>APRS,WIDE2-1:two");
    auto b3 = parse_ax25_packet("N0CALL>APRS,WIDE3-1:three");
    EXPECT_TRUE(digi.can_repeat(b1.data(), b1.size()) != nullptr);
    EXPECT_TRUE(digi.can_repeat(b2.data(), b2.size()) != nullptr);
    EXPECT_TRUE(digi.can_repeat(b3.data(), b3.size()) == nullptr);
    passed++;
}

TEST(n_N_class_mask_generic_wide_serves_all) {
    // A generic "WIDE" alias (all class bits set) serves classes 1..7.
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE", Alias::CLASS_MASK);
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE7-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result != nullptr);
    passed++;
}

// ============================================================================
// Operator-classification cross-product (ADR-0008): the firmware classifies
// an alias by its operator-marked class mask, NOT by string syntax.  The
// conformance generator can only express syntax-derived classification
// (routes.json), so these hand-written vectors exercise the operator model
// directly via make_alias(call, classes).
// ============================================================================

TEST(classification_relay_operator_nN) {
    // Operator marks RELAY as an n-N alias (class 2) even though "RELAY"
    // has no trailing class digit and would be explicit under syntax rules.
    // The firmware must serve it as n-N when the class mask says so.
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("RELAY", 0x04);  // operator: n-N class 2 (bit 2)
    TestDigipeater digi(cfg);
    auto buf = parse_ax25_packet("N0CALL>APRS,RELAY2-2:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());
    EXPECT_TRUE(result != nullptr);
    passed++;
}

TEST(classification_wide_operator_explicit) {
    // Operator marks plain WIDE as an explicit alias (classes == 0), not
    // n-N.  It must match the full callsign "WIDE" only, and must NOT match
    // the n-N wire form WIDE1-1.
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE", 0);  // operator: explicit
    TestDigipeater digi(cfg);
    auto b_explicit = parse_ax25_packet("N0CALL>APRS,WIDE:hello");
    auto b_nN = parse_ax25_packet("N0CALL>APRS,WIDE1-1:hello");
    EXPECT_TRUE(digi.can_repeat(b_explicit.data(), b_explicit.size()) != nullptr);
    EXPECT_TRUE(digi.can_repeat(b_nN.data(), b_nN.size()) == nullptr);
    passed++;
}

// ============================================================================
// Self-routing (deficiency A): when the first unused path address is our own
// callsign (SSID-aware), digipeat it -- mark the H-bit, no insert (it is
// already ours).  Positive coverage for conformance cases 76-83, 84, 85, 89,
// 90, 94, 102, 147, 201, 215.
// ============================================================================

TEST(self_route_mycall_first_unused_marks_only) {
    // mycall == first unused address -> mark, no insert/duplicate.
    auto cfg = make_test_config("ROUTER", 0);
    TestDigipeater digi(cfg);
    auto in_buf = parse_ax25_packet("DIGI>APRS,ROUTER:data");
    auto match = digi.can_repeat(in_buf.data(), in_buf.size());
    EXPECT_TRUE(match != nullptr);

    std::array<uint8_t, 330> out{};
    size_t len = 0;
    EXPECT_TRUE(digi.rewrite_frame(in_buf.data(), in_buf.size(), out.data(), len, out.size()));
    auto str = ax25_packet_to_string(out.data(), len);
    EXPECT_TRUE(str.find("DIGI>APRS,ROUTER*") != std::string::npos);
    EXPECT_TRUE(str.find("ROUTER,ROUTER") == std::string::npos);  // no duplicate insert
    passed++;
}

TEST(self_route_mycall_ssid_aware) {
    // mycall DIGI1-1 (SSID 1) matches DIGI1-1 at first unused.
    auto cfg = make_test_config("DIGI1-1", 0);
    TestDigipeater digi(cfg);
    auto in_buf = parse_ax25_packet("N0CALL>APRS,DIGI1-1:data");
    auto match = digi.can_repeat(in_buf.data(), in_buf.size());
    EXPECT_TRUE(match != nullptr);
    std::array<uint8_t, 330> out{};
    size_t len = 0;
    EXPECT_TRUE(digi.rewrite_frame(in_buf.data(), in_buf.size(), out.data(), len, out.size()));
    auto str = ax25_packet_to_string(out.data(), len);
    EXPECT_TRUE(str.find("DIGI1-1*") != std::string::npos);
    passed++;
}

TEST(self_route_does_not_fire_for_wrong_ssid) {
    // mycall DIGI1-1; path address DIGI1-2 (SSID 2) is a different station.
    // No alias configured -> no route.
    auto cfg = make_test_config("DIGI1-1", 0);
    TestDigipeater digi(cfg);
    auto in_buf = parse_ax25_packet("N0CALL>APRS,DIGI1-2:data");
    EXPECT_TRUE(digi.can_repeat(in_buf.data(), in_buf.size()) == nullptr);
    passed++;
}

// ============================================================================
// Alias representability (requirement 2026-08-18): an n-N alias whose prefix
// fills the whole 6-character callsign field is unrepresentable on the wire.
// The KISS set-alias command zeroes the config byte for such an alias.
// ============================================================================

TEST(alias_unrepresentable_nN_6char_prefix) {
    auto a = make_alias("FOOBAR", 0x04);  // n-N class 2, 6-char prefix
    EXPECT_TRUE(a.is_unrepresentable_nN());
    EXPECT_TRUE(a.callsign_len() == 6);
    passed++;
}

TEST(alias_representable_nN_short_prefix) {
    auto a = make_alias("WIDE", 0x04);    // 4-char prefix fits WIDE2
    EXPECT_FALSE(a.is_unrepresentable_nN());
    passed++;
}

TEST(alias_explicit_never_unrepresentable) {
    auto a = make_alias("CALLD", 0);      // explicit, mask 0
    EXPECT_FALSE(a.is_unrepresentable_nN());
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
