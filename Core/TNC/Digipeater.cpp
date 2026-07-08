// Copyright 2026 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#include "Digipeater.hpp"
#include "IOEventTask.h"
#include "HdlcFrame.hpp"
#include "KissHardware.hpp"

#include <cstring>
#include <cstdio>

extern osMessageQId hdlcOutputQueueHandle;

/*
 * APRS Digipeater implementation.
 */

// Beacon timer callback context
struct BeaconContext {
    uint8_t slot;                    // Beacon slot index (0-3)
    mobilinkd::tnc::Digipeater* digi;
};

static BeaconContext beacon_contexts[4];
static osTimerId beaconTimerHandles[4];

static void beaconTimerCallback(void const* arg)
{
    using namespace mobilinkd::tnc;
    using namespace mobilinkd::tnc::hdlc;
    using mobilinkd::tnc::kiss::settings;
    using mobilinkd::tnc::kiss::NUMBER_OF_BEACONS;

    auto ctx = static_cast<const BeaconContext*>(arg);
    auto& beacon = settings().beacons[ctx->slot];

    if (beacon.seconds == 0) return; // Slot not configured

    auto frame = hdlc::acquire();
    if (frame == nullptr) {
        ERROR("Beacon: OOM");
        return;
    }

    // Destination address: 6 shifted chars + SSID byte.
    // call_t is char[8] NUL-padded; treat NUL as space for AX.25.
    for (size_t i = 0; i < 6; i++) {
        char c = beacon.dest[i] ? beacon.dest[i] : ' ';
        frame->push_back(static_cast<uint8_t>(c << 1));
    }
    // C-bit: 0 if path follows, 1 if direct (no path).
    frame->push_back(beacon.path_count > 0 ? 0x00 : 0x01);

    // Source address (mycall): 6 shifted chars + SSID byte.
    auto& mycall = settings().mycall;
    for (size_t i = 0; i < 6; i++) {
        frame->push_back(static_cast<uint8_t>(mycall[i] << 1));
    }
    frame->push_back(beacon.path_count > 0 ? 0x00 : 0x01);

    // Pre-encoded path addresses — pure byte copy, no string parsing.
    for (uint8_t i = 0; i < beacon.path_count; i++) {
        bool is_last = (i == beacon.path_count - 1);
        for (int j = 0; j < 6; j++) {
            frame->push_back(beacon.path[i][j]);
        }
        // C-bit: set on last address, clear otherwise.
        uint8_t ssid_byte = beacon.path[i][6] | (is_last ? 0x01 : 0x00);
        frame->push_back(ssid_byte);
    }

    // Control field: UI frame
    frame->push_back(0x03);

    // PID: No layer 3 protocol
    frame->push_back(0xF0);

    // Information field: beacon text
    for (size_t i = 0; i < beacon.text_len; i++) {
        if (!frame->push_back(beacon.text[i])) {
            ERROR("Beacon: OOM pushing text");
            hdlc::release(frame);
            return;
        }
    }

    // Add FCS
    frame->add_fcs();

    // Post to modulator queue using p-persist CSMA (standard).
    if (osMessagePut(hdlcOutputQueueHandle,
        reinterpret_cast<uint32_t>(frame),
        0) != osOK) {
        ERROR("Beacon: failed to post to TX queue");
        hdlc::release(frame);
    }
}

void start_beacon_timers()
{
    using namespace mobilinkd::tnc::kiss;

    for (size_t i = 0; i < NUMBER_OF_BEACONS; i++) {
        beacon_contexts[i].slot = i;
        beacon_contexts[i].digi = nullptr;

        osTimerDef(beaconTimer, beaconTimerCallback);
        beaconTimerHandles[i] = osTimerCreate(osTimer(beaconTimer), osTimerPeriodic, &beacon_contexts[i]);

        if (beaconTimerHandles[i] && settings().beacons[i].seconds > 0) {
            osTimerStart(beaconTimerHandles[i], settings().beacons[i].seconds * 1000);
        }
    }
}

void startDigipeaterTask(void* arg)
{
  using mobilinkd::tnc::Digipeater;
  using mobilinkd::tnc::hdlc::IoFrame;
  using mobilinkd::tnc::hdlc::TxResult;
  using mobilinkd::tnc::hdlc::add_ref;
  using mobilinkd::tnc::hdlc::release;

  // Create static Digipeater instance using settings from EEPROM
  static Digipeater digi_instance(mobilinkd::tnc::kiss::settings().aliases, mobilinkd::tnc::kiss::settings().beacons);

  auto digi = arg ? static_cast<Digipeater*>(arg) : &digi_instance;

  // Start beacon timers
  start_beacon_timers();

  for(;;)
  {
    osEvent evt = osMessageGet(digipeaterQueueHandle, osWaitForever);
    if (evt.status != osEventMessage) continue;

    uint32_t cmd = evt.value.v;
    if (cmd < FLASH_BASE) // Assumes FLASH_BASE < SRAM_BASE.
    {
      // this is a command, not a packet.
      return;
    }

    auto frame = static_cast<IoFrame*>(evt.value.p);

    // Check if this is a TX completion notification
    if (frame->tx_result() != TxResult::NONE)
    {
        // Frame came back from modulator -- TX is complete.
        release(frame);  // Release digipeater's reference
        continue;
    }

    digi->clean_history();

    if (!digi->can_repeat(frame)) {
        release(frame);
        continue;
    }

    auto digi_frame = digi->rewrite_frame(frame);
    if (digi_frame == frame) {
        // rewrite_frame returned the original -- router declined it
        release(frame);
        continue;
    }

    // Release original frame -- digipeater is done with it
    release(frame);

    add_ref(digi_frame);  // Digipeater holds a reference for TX completion tracking
    digi_frame->tx_completion_queue(digipeaterQueueHandle);
    osMessagePut(hdlcOutputQueueHandle,
        reinterpret_cast<uint32_t>(digi_frame),
        osWaitForever);

  }
}

void beacon(void* arg)
{
  (void)arg;
  // Legacy entry point. Beacon scheduling is now handled by FreeRTOS timers
  // started in startDigipeaterTask().
}

namespace mobilinkd { namespace tnc {

}}  // mobilinkd::tnc
