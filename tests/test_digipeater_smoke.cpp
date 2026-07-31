// Smoke tests for the digipeater buffer-based routing core.
// Verifies the test harness works and basic n-N routing is correct.

#include "test_digipeater_harness.hpp"

#include <iostream>
#include <cassert>
#include <cstring>

using namespace mobilinkd::tnc::kiss;
using namespace test;

static int passed = 0;
static int failed = 0;

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
// Test 1: Parse an APRS packet and verify roundtrip
// ============================================================================
TEST(parse_roundtrip) {
    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE1-1,WIDE2-2:test data");
    EXPECT(buf.size() > 14, "buffer too small");

    auto str = ax25_packet_to_string(buf, buf.size());
    // Should contain key elements
    EXPECT(str.find("N0CALL") != std::string::npos, "missing source");
    EXPECT(str.find("APRS") != std::string::npos, "missing destination");
    EXPECT(str.find("WIDE1-1") != std::string::npos, "missing WIDE1-1");
    EXPECT(str.find("WIDE2-2") != std::string::npos, "missing WIDE2-2");
    EXPECT(str.find("test data") != std::string::npos, "missing info");
    passed++;
}

// ============================================================================
// Test 2: WIDE1-1 alias matches WIDE1-1 in path (can_repeat)
// ============================================================================
TEST(can_repeat_wide1_1_match) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);

    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());

    EXPECT_TRUE(result != nullptr);
    EXPECT(result->hops == 1, "alias hops should be 1");
    passed++;
}

// ============================================================================
// Test 3: WIDE2-2 alias matches WIDE2-2 in path
// ============================================================================
TEST(can_repeat_wide2_2_match) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE2", 2);
    TestDigipeater digi(cfg);

    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE2-2:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());

    EXPECT_TRUE(result != nullptr);
    passed++;
}

// ============================================================================
// Test 4: Frame addressed to our callsign is rejected
// ============================================================================
TEST(can_repeat_reject_self_addressed) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);

    auto buf = parse_ax25_packet("N0CALL>DIGI,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());

    EXPECT_TRUE(result == nullptr);
    passed++;
}

// ============================================================================
// Test 5: Frame already repeated by us is rejected
// ============================================================================
TEST(can_repeat_reject_already_repeated) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);

    // DIGI* with H-bit set means we already repeated this
    auto buf = parse_ax25_packet("N0CALL>APRS,DIGI*,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());

    EXPECT_TRUE(result == nullptr);
    passed++;
}

// ============================================================================
// Test 6: rewrite_frame: WIDE1-1 → DIGI* (substitution always on)
// ============================================================================
TEST(rewrite_wide1_1_substitute) {
    auto cfg = make_test_config("DIGI", 0);  // no routing flags needed
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

    auto str = ax25_packet_to_string(std::vector<uint8_t>(out_buf.data(), out_buf.data() + out_len), out_len);
    // Substitution is hardcoded: WIDE1-1 exhausted → replaced by DIGI*
    std::cerr << "  output: " << str << "\n";
    EXPECT(str.find("DIGI*") != std::string::npos, "DIGI* not found in output: " + str);
    EXPECT(str.find("WIDE1") == std::string::npos, "WIDE1 should be gone: " + str);
    passed++;
}

// ============================================================================
// Test 8: WIDE2-2 rewrite produces WIDE2-1 with DIGI inserted
// ============================================================================
TEST(rewrite_wide2_2_to_wide2_1) {
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE2", 2);
    TestDigipeater digi(cfg);

    auto in_buf = parse_ax25_packet("N0CALL>APRS,WIDE2-2:hello");
    auto alias = digi.can_repeat(in_buf.data(), in_buf.size());
    EXPECT_TRUE(alias != nullptr);

    std::array<uint8_t, 330> out_buf{};
    size_t out_len = 0;
    auto ok = digi.rewrite_frame(in_buf.data(), in_buf.size(),
                                 out_buf.data(), out_len, out_buf.size());
    EXPECT_TRUE(ok);

    auto str = ax25_packet_to_string(std::vector<uint8_t>(out_buf.data(), out_buf.data() + out_len), out_len);
    EXPECT(str.find("DIGI") != std::string::npos, "DIGI not found: " + str);
    EXPECT(str.find("WIDE2-1") != std::string::npos, "WIDE2-1 not found: " + str);
    passed++;
}

// ============================================================================
// Test 9: reject WIDE2-0 (SSID already 0)
// ============================================================================
TEST(can_repeat_reject_ssid_zero) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE2", 2);
    TestDigipeater digi(cfg);

    auto buf = parse_ax25_packet("N0CALL>APRS,WIDE2-0:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());

    EXPECT_TRUE(result == nullptr);
    passed++;
}

// ============================================================================
// Test 10: non-APRS frame is rejected
// ============================================================================
TEST(can_repeat_reject_non_aprs) {
    auto cfg = make_test_config("DIGI");
    cfg.aliases[0] = make_alias("WIDE1", 1);
    TestDigipeater digi(cfg);

    // Destination "N0CALL" — not an APRS TOCALL
    auto buf = parse_ax25_packet("N0CALL>N0CALL,WIDE1-1:hello");
    auto result = digi.can_repeat(buf.data(), buf.size());

    EXPECT_TRUE(result == nullptr);
    passed++;
}

int main() {
    std::cout << "Digipeater Smoke Tests\n";
    std::cout << "======================\n\n";

    // Tests run via static constructors above

    std::cout << "\n";
    std::cout << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}
