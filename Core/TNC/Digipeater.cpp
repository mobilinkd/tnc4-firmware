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
    using mobilinkd::tnc::kiss::Beacon;
    using mobilinkd::tnc::kiss::settings;
    using mobilinkd::tnc::kiss::NUMBER_OF_BEACONS;

    auto ctx = static_cast<const BeaconContext*>(arg);
    auto& beacon = settings().beacons[ctx->slot];

    if (beacon.seconds == 0) return; // Slot not configured

    // Build an AX.25 frame: dest(7) + src(7) + path + ctrl(0x03) + pid(0xF0) + info
    // Total must fit in an IoFrame

    auto frame = hdlc::acquire();
    if (frame == nullptr) {
        ERROR("Beacon: OOM");
        return;
    }

    // Encode destination (7 bytes: 6 char + SSID, shifted left by 1)
    for (size_t i = 0; i < 6; i++) {
        char c = (i < strnlen(beacon.dest.data(), 6)) ? beacon.dest[i] : ' ';
        frame->push_back(c << 1);
    }
    frame->push_back(0x00); // no SSID, not last

    // Encode source (MYCALL, 7 bytes)
    auto& mycall = settings().mycall;
    for (size_t i = 0; i < 6; i++) {
        frame->push_back(mycall[i] << 1);
    }
    frame->push_back(0x00); // no SSID, not last yet

    // Encode digipeater path from beacon settings (comma-separated)
    const uint8_t* path_ptr = beacon.path;
    size_t path_len = strnlen((const char*)path_ptr, kiss::BEACON_PATH_LEN);
    size_t addr_idx = 0;
    size_t path_pos = 0;

    while (path_pos < path_len && addr_idx < 8) {
        // Extract address up to comma or end
        char addr_buf[10] = {};
        size_t addr_len = 0;
        while (path_pos + addr_len < path_len && path_ptr[path_pos + addr_len] != ',' && addr_len < 8) {
            addr_buf[addr_len] = path_ptr[path_pos + addr_len];
            addr_len++;
        }
        addr_buf[addr_len] = '\0';

        bool last_addr = (path_pos + addr_len >= path_len);

        // Parse CALLSIGN-N format
        char call_part[7] = {};
        int ssid = 0;
        const char* dash = std::strchr(addr_buf, '-');
        if (dash && (dash - addr_buf) < 7) {
            size_t call_len = dash - addr_buf;
            std::strncpy(call_part, addr_buf, call_len);
            call_part[call_len] = '\0';
            ssid = std::atoi(dash + 1);
        } else {
            size_t copy_len = std::min(strlen(addr_buf), size_t(6));
            for (size_t i = 0; i < copy_len; i++) call_part[i] = addr_buf[i];
            call_part[copy_len] = '\0';
        }

        // Encode to 7-byte shifted format
        size_t call_len = std::strlen(call_part);
        for (size_t i = 0; i < 6; i++) {
            char c = (i < call_len) ? call_part[i] : ' ';
            frame->push_back(c << 1);
        }
        uint8_t ssid_byte = (ssid << 1) | (last_addr ? 0x01 : 0x00);
        frame->push_back(ssid_byte);

        path_pos += addr_len + 1; // skip comma
        addr_idx++;
    }

    // Control field: UI frame = 0x03
    frame->push_back(0x03);

    // PID: No layer 3 protocol = 0xF0
    frame->push_back(0xF0);

    // Information field: beacon text
    size_t text_len = strnlen((const char*)beacon.text, kiss::BEACON_TEXT_LEN);
    for (size_t i = 0; i < text_len; i++) {
        if (!frame->push_back(beacon.text[i])) {
            ERROR("Beacon: OOM pushing text");
            hdlc::release(frame);
            return;
        }
    }

    // Add FCS
    frame->add_fcs();

    // Post to modulator queue
    // Beacons use p-persist CSMA (standard), not p=0 like digipeated frames
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
