// Copyright 2026 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.
//
// Firmware-side digipeater: platform policy + IoFrame glue declarations.
// The routing logic lives in DigipeaterCore.hpp (platform-independent).

#ifndef MOBILINKD__TNC__DIGIPEATER_HPP_
#define MOBILINKD__TNC__DIGIPEATER_HPP_

#include "DigipeaterCore.hpp"
#include "KissHardware.hpp"
#include "HdlcFrame.hpp"

#include <cmsis_os.h>

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
 * Firmware platform policy for DigipeaterCore.
 * Provides the monotonic clock via HAL_GetTick().
 */
struct FirmwarePolicy {
    uint32_t now_ms() { return HAL_GetTick(); }
};

/// The concrete digipeater type used in firmware.
using FirmwareDigipeater = DigipeaterCore<FirmwarePolicy, kiss::Hardware>;

// ============================================================================
// IoFrame glue -- defined in Digipeater.cpp
// ============================================================================

/**
 * Check if an IoFrame can be digipeated.
 * Copies frame data into the core's linear buffer via iterators,
 * then delegates to the buffer-based can_repeat().
 */
const kiss::Alias* digi_can_repeat(FirmwareDigipeater& digi, hdlc::IoFrame* frame);

/**
 * Rewrite an IoFrame for digipeating.
 * Returns a NEW IoFrame with the routed frame, or the original frame
 * if routing was declined.  Caller must release() the original.
 */
hdlc::IoFrame* digi_rewrite_frame(FirmwareDigipeater& digi, hdlc::IoFrame* frame);

}} // mobilinkd::tnc

#endif // __cplusplus

#endif // MOBILINKD__TNC__DIGIPEATER_HPP_
