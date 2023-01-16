// Copyright 2018-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AudioInput.hpp"
#include "Afsk1200Demodulator.hpp"
#include "Fsk9600Demodulator.hpp"
#include "M17Demodulator.h"
#include "AudioLevel.hpp"
#include "Log.h"
#include "KissHardware.hpp"
#include "GPIO.hpp"
#include "HdlcFrame.hpp"
#include "PortInterface.hpp"
#include "Goertzel.h"
#include "DCD.h"
#include "ModulatorTask.hpp"

#include "arm_math.h"
#include "stm32l4xx_hal.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <type_traits>

extern osMessageQId ioEventQueueHandle;
extern IWDG_HandleTypeDef hiwdg;

extern "C" void SystemClock_Config(void);

// DMA Conversion first half complete.
extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef*) {
    using namespace mobilinkd::tnc::audio;

    auto block = adcPool.allocate();
    if (!block) return;
    memmove(block->buffer, adc_buffer, dma_transfer_size);
    auto status = osMessagePut(adcInputQueueHandle, (uint32_t) block, 0);
    if (status != osOK) adcPool.deallocate(block);
}

// DMA Conversion second half complete.
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*) {
    using namespace mobilinkd::tnc::audio;

    auto block = adcPool.allocate();
    if (!block) return;
    memmove(block->buffer, adc_buffer + half_buffer_size, dma_transfer_size);
    auto status = osMessagePut(adcInputQueueHandle, (uint32_t) block, 0);
    if (status != osOK) adcPool.deallocate(block);
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* /* hadc */) {
    using namespace mobilinkd::tnc::audio;

    // __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
    // HAL_DMA_Start(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)adc_buffer, ADC_BUFFER_SIZE * 2);
}

extern "C" void startAudioInputTask(void const*) {

    using namespace mobilinkd::tnc::audio;
    TNC_DEBUG("startAudioInputTask");

    adcPool.init();

    uint8_t adcState = mobilinkd::tnc::audio::IDLE;

    while (true) {
        osEvent event = osMessageGet(audioInputQueueHandle, osWaitForever);
        if (event.status != osEventMessage) continue;
        adcState = event.value.v;

        switch (adcState) {
        case STOPPED:
            TNC_DEBUG("STOPPED");
            // stop();
            break;
        case DEMODULATOR:
            INFO("DEMODULATOR");
            demodulatorTask();
            break;
        case STREAM_AMPLIFIED_INPUT_LEVEL:
            TNC_DEBUG("STREAM_AMPLIFIED_INPUT_LEVEL");
            streamAmplifiedInputLevels();
            break;
        case POLL_AMPLIFIED_INPUT_LEVEL:
            TNC_DEBUG("POLL_AMPLIFIED_INPUT_LEVEL");
            pollAmplifiedInputLevel();
            break;
#ifndef NUCLEOTNC
        case POLL_BATTERY_LEVEL:
            TNC_DEBUG("POLL_BATTERY_LEVEL");
            pollBatteryLevel();
            break;
#endif
        case POLL_TWIST_LEVEL:
            TNC_DEBUG("POLL_TWIST_LEVEL");
            pollInputTwist();
            break;
        case STREAM_AVERAGE_TWIST_LEVEL:
            TNC_DEBUG("STREAM_AVERAGE_TWIST_LEVEL");
            // streamAverageInputTwist();
            break;
        case STREAM_INSTANT_TWIST_LEVEL:
            TNC_DEBUG("STREAM_INSTANT_TWIST_LEVEL");
            // streamInstantInputTwist();
            break;
        case AUTO_ADJUST_INPUT_LEVEL:
            TNC_DEBUG("AUTO_ADJUST_INPUT_LEVEL");
            autoAudioInputLevel();
            break;
        case CONFIGURE_INPUT_LEVELS:
            TNC_DEBUG("CONFIGURE_INPUT_LEVELS");
            setAudioInputLevels();
            break;
        case UPDATE_SETTINGS:
            TNC_DEBUG("UPDATE_SETTINGS");
            setAudioInputLevels();
            updateModulator();
            break;
        case IDLE:
            INFO("IDLE");
            break;
        default:
            break;
        }
    }
}

namespace mobilinkd { namespace tnc { namespace audio {

uint32_t adc_buffer[ADC_BUFFER_SIZE];               // Two samples per element.
volatile uint32_t adc_block_size = ADC_BUFFER_SIZE;          // Based on demodulator.
volatile uint32_t dma_transfer_size = adc_block_size * 2;    // Transfer size in bytes.
volatile uint32_t half_buffer_size = adc_block_size / 2;     // Transfer size in words / 2.
adc_pool_type adcPool;

void set_adc_block_size(uint32_t block_size)
{
    adc_block_size = block_size;
    dma_transfer_size = block_size * 2;
    half_buffer_size = block_size / 2;
}

q15_t normalized[ADC_BUFFER_SIZE];

IDemodulator* getDemodulator()
{
    constexpr auto mem_size = std::max({
        sizeof(Afsk1200Demodulator),
        sizeof(Fsk9600Demodulator),
        sizeof(M17Demodulator),
    });

    using storage_t = std::aligned_storage<mem_size, 4>::type;

    static storage_t mem;
    static IDemodulator* demod = nullptr;

    static uint8_t modem_type = 0;
    if (modem_type != kiss::settings().modem_type)
    {
        if (demod)
        {
            demod->~IDemodulator();
        }

        switch (kiss::settings().modem_type)
        {
        case kiss::Hardware::ModemType::AFSK1200:
            demod = new (&mem) Afsk1200Demodulator();
            break;
        case kiss::Hardware::ModemType::FSK9600:
            demod = new (&mem) Fsk9600Demodulator();
            break;
        case kiss::Hardware::ModemType::M17:
            demod = new (&mem) M17Demodulator();
            break;
        default:
            ERROR("Invalid demodulator");
            CxxErrorHandler();
        }
        modem_type = kiss::settings().modem_type;
    }

    return demod;
}

void demodulatorTask() {

    TNC_DEBUG("enter demodulatorTask");

    bool dcd_status{false};
    auto demodulator = getDemodulator();

    demodulator->start();

    while (true) {
        osEvent peek = osMessagePeek(audioInputQueueHandle, 0);
        if (peek.status == osEventMessage) break;

        osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
        if (evt.status != osEventMessage) {
            continue;
        }

        HAL_IWDG_Refresh(&hiwdg);

        auto block = (adc_pool_type::chunk_type*) evt.value.p;
        auto samples = (int16_t*) block->buffer;

        arm_offset_q15(samples, 0 - virtual_ground, normalized, demodulator->size());
        adcPool.deallocate(block);

        auto frame = (*demodulator)(normalized);
        if (frame)
        {
            frame->source(frame->source() | hdlc::IoFrame::RF_DATA);
            if (osMessagePut(ioEventQueueHandle, (uint32_t) frame, 1) != osOK)
            {
                hdlc::release(frame);
            }
        }

        if (demodulator->locked() xor dcd_status) {
            dcd_status = demodulator->locked();
            if (dcd_status) {
                dcd_on();
            } else {
                dcd_off();
            }
        }
    }

    demodulator->stop();

    dcd_off();
    TNC_DEBUG("exit demodulatorTask");
}


void streamLevels(uint8_t cmd) {

    // Stream out Vpp, Vavg, Vmin, Vmax as four 16-bit values, left justified.

    constexpr uint32_t BLOCKS = 30;

    uint8_t data[9];
    INFO("streamLevels: start");

    auto demodulator = getDemodulator();
    demodulator->start();
    const size_t audio_exponent = demodulator->get_adc_exponent();

    while (true) {
        osEvent peek = osMessagePeek(audioInputQueueHandle, 0);
        if (peek.status == osEventMessage) break;

        uint32_t count = 0;
        uint32_t accum = 0;
        uint16_t vmin = std::numeric_limits<uint16_t>::max();
        uint16_t vmax = std::numeric_limits<uint16_t>::min();

        for (size_t i = 0; i != BLOCKS; ++i) {
            osEvent evt = osMessageGet(adcInputQueueHandle, 1000);
            if (evt.status != osEventMessage) break;

            count += demodulator->size();

            auto block = (adc_pool_type::chunk_type*) evt.value.p;
            auto start =  (uint16_t*) block->buffer;
            auto end = start + demodulator->size();

            vmin = std::min(vmin, *std::min_element(start, end));
            vmax = std::max(vmax, *std::max_element(start, end));
            accum = std::accumulate(start, end, accum);

            adcPool.deallocate(block);
        }

        uint16_t pp = (vmax - vmin) << audio_exponent;
        uint16_t avg = (accum / count) << audio_exponent;
        vmin <<= audio_exponent;
        vmax <<= audio_exponent;

        data[0] = cmd;
        data[1] = (pp >> 8) & 0xFF;   // Vpp
        data[2] = (pp & 0xFF);
        data[3] = (avg >> 8) & 0xFF;  // Vavg (DC level)
        data[4] = (avg & 0xFF);
        data[5] = (vmin >> 8) & 0xFF;  // Vmin
        data[6] = (vmin & 0xFF);
        data[7] = (vmax >> 8) & 0xFF;  // Vmax
        data[8] = (vmax & 0xFF);

        ioport->write(data, 9, 6, 10);
    }

    demodulator->stop();
    TNC_DEBUG("exit streamLevels");
}

levels_type readLevels(uint32_t)
{

    TNC_DEBUG("enter readLevels");

    // Return Vpp, Vavg, Vmin, Vmax as four 16-bit values, right justified.

    uint32_t BLOCKS = 30;
    uint32_t accum = 0;
    uint32_t iaccum = 0;
    uint16_t vmin = std::numeric_limits<uint16_t>::max();
    uint16_t vmax = std::numeric_limits<uint16_t>::min();

    INFO("readLevels: start");
    auto demodulator = getDemodulator();
    demodulator->start();

    for (uint32_t count = 0; count != BLOCKS; ++count)
    {
        osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
        if (evt.status != osEventMessage) continue;

        auto block = (adc_pool_type::chunk_type*) evt.value.p;
        auto start =  (uint16_t*) block->buffer;
        auto end = start + demodulator->size();

        vmin = std::min(vmin, *std::min_element(start, end));
        vmax = std::max(vmax, *std::max_element(start, end));
        accum = std::accumulate(start, end, accum);

        iaccum += (accum / demodulator->size());

        adcPool.deallocate(block);

        accum = 0;
    }

    demodulator->stop();

    uint16_t pp = vmax - vmin;
    uint16_t avg = iaccum / BLOCKS;
    INFO("exit readLevels");

    return levels_type(pp, avg, vmin, vmax);
}


/**
 * This provides 100Hz resolution to the Goerztel filter.
 */
constexpr uint32_t TWIST_SAMPLE_SIZE = 88;

/*
 * Return twist as a the difference in dB between mark and space.  The
 * expected values are about 0dB for discriminator output and about 5.5dB
 * for de-emphasized audio.
 */
float readTwist()
{
    return getDemodulator()->readTwist();
}

/*
 * Get the input twist level as a pair of numbers -- the relative dB
 * level of the Bell 202 mark and space tones.
 *
 * This is intended to measure noise levels on an empty channel.
 *
 * When de-emphasis is applied, the noise at 1200Hz will be about 5.5dB
 * higher than at 2200Hz.  When de-emphasis is not applied (discriminator
 * output), the levels should be about the same.
 *
 * This is used to adjust the demodulator filters so that the proper
 * input twist is applied to the signal.  In general, properly modulated
 * signals are expected to be pre-emphasized so that they are equal
 * when de-emphasis is applied.
 *
 * If no de-emphasis is detected, the de-emphasis has to be applied in
 * the demodulator.
 *
 * This takes about 5 seconds to complete as it averages 100 50ms samples
 * to get a reasonable sampling of the noise.
 */
void pollInputTwist()
{
    TNC_DEBUG("enter pollInputTwist");

    float g1200 = 0.0f;
    float g2200 = 0.0f;

    GoertzelFilter<TWIST_SAMPLE_SIZE, SAMPLE_RATE> gf1200(1200.0);
    GoertzelFilter<TWIST_SAMPLE_SIZE, SAMPLE_RATE> gf2200(2200.0);

    const uint32_t AVG_SAMPLES = 100;

    IDemodulator::startADC(1817, TWIST_SAMPLE_SIZE);

    for (uint32_t i = 0; i != AVG_SAMPLES; ++i) {

      uint32_t count = 0;
      while (count < TWIST_SAMPLE_SIZE) {

          osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
          if (evt.status != osEventMessage) continue;

          count += ADC_BUFFER_SIZE;

          auto block = (adc_pool_type::chunk_type*) evt.value.p;
          uint16_t* data =  (uint16_t*) block->buffer;
          gf1200(data, ADC_BUFFER_SIZE);
          gf2200(data, ADC_BUFFER_SIZE);

          adcPool.deallocate(block);
      }

      g1200 += 10.0 * log10(gf1200);
      g2200 += 10.0 * log10(gf2200);

      gf1200.reset();
      gf2200.reset();
    }

    IDemodulator::stopADC();

    TNC_DEBUG("pollInputTwist: MARK=%d, SPACE=%d (x100)",
      int(g1200 * 100.0 / AVG_SAMPLES), int(g2200 * 100.0 / AVG_SAMPLES));

    int16_t g1200i = int16_t(g1200 * 256 / AVG_SAMPLES);
    int16_t g2200i = int16_t(g2200 * 256 / AVG_SAMPLES);

    uint8_t buffer[5];
    buffer[0] = kiss::hardware::POLL_INPUT_TWIST;
    buffer[1] = (g1200i >> 8) & 0xFF;
    buffer[2] = g1200i & 0xFF;
    buffer[3] = (g2200i >> 8) & 0xFF;
    buffer[4] = g2200i & 0xFF;

    ioport->write(buffer, 5, 6, 10);

    TNC_DEBUG("exit pollInputTwist");
}

void streamAmplifiedInputLevels() {
    TNC_DEBUG("enter streamAmplifiedInputLevels");
    streamLevels(kiss::hardware::POLL_INPUT_LEVEL);
    TNC_DEBUG("exit streamAmplifiedInputLevels");
}

void pollAmplifiedInputLevel() {
    TNC_DEBUG("enter pollAmplifiedInputLevel");

    uint16_t Vpp, Vavg, Vmin, Vmax;
    std::tie(Vpp, Vavg, Vmin, Vmax) = readLevels(AUDIO_IN);

    const size_t audio_exponent = getDemodulator()->get_adc_exponent();

    Vpp <<= audio_exponent;
    Vavg <<= audio_exponent;
    Vmin <<= audio_exponent;
    Vmax <<= audio_exponent;

    uint8_t data[9];
    data[0] = kiss::hardware::POLL_INPUT_LEVEL;
    data[1] = (Vpp >> 8) & 0xFF;   // Vpp
    data[2] = (Vpp & 0xFF);
    data[3] = (Vavg >> 8) & 0xFF;  // Vavg (DC level)
    data[4] = (Vavg & 0xFF);
    data[5] = (Vmin >> 8) & 0xFF;  // Vmin
    data[6] = (Vmin & 0xFF);
    data[7] = (Vmax >> 8) & 0xFF;  // Vmax
    data[8] = (Vmax & 0xFF);

    ioport->write(data, 9, 6, 10);
    TNC_DEBUG("exit pollAmplifiedInputLevel");
}

#ifndef NUCLEOTNC
void pollBatteryLevel()
{
    auto vbat = getDemodulator()->readBatteryLevel();

    uint8_t data[3];
    data[0] = kiss::hardware::GET_BATTERY_LEVEL;
    data[1] = (vbat >> 8) & 0xFF;
    data[2] = (vbat & 0xFF);

    ioport->write(data, 3, 6, 10);
}
#endif

#if 0
void stop() {
    osDelay(100);
    auto restore = SysTick->CTRL;

    kiss::settings().input_offset += 6;
    setAudioInputLevels();
    kiss::settings().input_offset -= 6;
    TNC_DEBUG("Stop");
    // __disable_irq();
    vTaskSuspendAll();
    SysTick->CTRL = 0;
    HAL_COMP_Init(&hcomp1);
    HAL_COMP_Start_IT(&hcomp1);
    while (adcState == STOPPED) {
        // PWR_MAINREGULATOR_ON / PWR_LOWPOWERREGULATOR_ON
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    }
    SystemClock_Config();
    SysTick->CTRL = restore;
    // __enable_irq();
    HAL_COMP_Stop_IT(&hcomp1);
    HAL_COMP_DeInit(&hcomp1);
    xTaskResumeAll();
    setAudioInputLevels();
    // adcState = DEMODULATOR;
    TNC_DEBUG("Wake");
}
#endif

}}} // mobilinkd::tnc::audio
