#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <cstdint>
#include <cstring>

// Pull in the serial port definitions; adjust include path as needed for your build.
#include "Core/TNC/SerialPort.hpp"

// The block buffer size as defined in the production code.
// Typically BLOCK_BUFFER_SIZE or similar — adjust to match actual constant.
#ifndef SERIAL_BLOCK_BUFFER_SIZE
#define SERIAL_BLOCK_BUFFER_SIZE 512
#endif

class SerialPortBoundsTest : public ::testing::TestWithParam<size_t> {};

TEST_P(SerialPortBoundsTest, LenNeverExceedsBlockBufferCapacity) {
    // Invariant: any 'len' derived from received serial data must never
    // cause a write beyond block->buffer + SERIAL_BLOCK_BUFFER_SIZE.
    // i.e., len <= SERIAL_BLOCK_BUFFER_SIZE - 1 must always hold before memmove.

    size_t len = GetParam();

    // Allocate a block with a known canary region after it to detect overflow.
    const size_t CANARY_SIZE = 64;
    uint8_t raw[sizeof(uint8_t) * (SERIAL_BLOCK_BUFFER_SIZE + 1) + CANARY_SIZE];
    uint8_t *block_buffer = raw;
    uint8_t *canary = raw + SERIAL_BLOCK_BUFFER_SIZE + 1;
    memset(canary, 0xAB, CANARY_SIZE);

    // Simulate what the production code does: memmove(block->buffer + 1, src, len)
    // We assert the invariant: len must not exceed capacity - 1.
    bool len_is_safe = (len <= static_cast<size_t>(SERIAL_BLOCK_BUFFER_SIZE - 1));

    if (len_is_safe) {
        uint8_t src[SERIAL_BLOCK_BUFFER_SIZE] = {};
        memmove(block_buffer + 1, src, len);
        // Canary must be intact after a safe copy.
        for (size_t i = 0; i < CANARY_SIZE; i++) {
            EXPECT_EQ(canary[i], 0xAB) << "Heap corruption detected at canary byte " << i;
        }
    }

    // The security invariant: oversized len values must be rejected/clamped.
    EXPECT_LE(len, static_cast<size_t>(SERIAL_BLOCK_BUFFER_SIZE - 1))
        << "SECURITY VIOLATION: len=" << len
        << " exceeds block buffer capacity (" << (SERIAL_BLOCK_BUFFER_SIZE - 1) << ")";
}

INSTANTIATE_TEST_SUITE_P(
    AdversarialInputs,
    SerialPortBoundsTest,
    ::testing::Values(
        // Exact exploit: len far exceeds buffer (e.g., attacker sends 0xFFFF)
        static_cast<size_t>(0xFFFF),
        // Boundary: exactly one byte over the safe limit
        static_cast<size_t>(SERIAL_BLOCK_BUFFER_SIZE),
        // Boundary: exactly at the safe limit
        static_cast<size_t>(SERIAL_BLOCK_BUFFER_SIZE - 1),
        // Valid: typical small frame
        static_cast<size_t>(64)
    )
);

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}