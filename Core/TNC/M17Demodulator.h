// Copyright 2020-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Demodulator.hpp"
#include "AudioLevel.hpp"
#include "AudioInput.hpp"
#include "ClockRecovery.h"
#include "DataCarrierDetect.h"
#include "FreqDevEstimator.h"
#include "GPIO.hpp"
#include "KissHardware.hpp"
#include "Log.h"
#include "M17.h"
#include "M17Framer.h"
#include "M17FrameDecoder.h"
#include "ModulatorTask.hpp"
#include "Modulator.hpp"
#include "SymbolEvm.h"
#include "Util.h"

#include <arm_math.h>

#include <algorithm>
#include <array>
#include <experimental/array>
#include <optional>
#include <tuple>

namespace mobilinkd { namespace tnc {

struct M17Demodulator : IDemodulator
{
    static constexpr uint32_t ADC_BLOCK_SIZE = 192;
    static_assert(audio::ADC_BUFFER_SIZE >= ADC_BLOCK_SIZE);

    static constexpr auto evm_b = std::experimental::make_array<float>(0.02008337, 0.04016673, 0.02008337);
    static constexpr auto evm_a = std::experimental::make_array<float>(1.0, -1.56101808, 0.64135154);

    static constexpr uint32_t SAMPLE_RATE = 48000;
    static constexpr uint32_t SYMBOL_RATE = 4800;
    static constexpr uint32_t SAMPLES_PER_SYMBOL = SAMPLE_RATE / SYMBOL_RATE;
    static constexpr uint16_t VREF = 16383;

    static constexpr float sample_rate = SAMPLE_RATE;
    static constexpr float symbol_rate = SYMBOL_RATE;

    using audio_filter_t = FirFilter<ADC_BLOCK_SIZE, m17::FILTER_TAP_NUM_15>;
    using demod_result_t = std::tuple<float, float, int, float>;

    enum class DemodState { UNLOCKED, LSF_SYNC, FRAME, STREAM_SYNC, PACKET_SYNC };

    audio_filter_t demod_filter;
    std::array<int8_t, 368> buffer;
    float evm_average = 0.0;
    m17::FreqDevEstimator<float> dev;
    SymbolEvm<float, std::tuple_size<decltype(evm_b)>::value> symbol_evm{evm_b, evm_a};
    m17::DataCarrierDetect<float, SAMPLE_RATE, ADC_BLOCK_SIZE, 1000> dcd{2000, 4000, 5.0};
    m17::ClockRecovery<float, SAMPLE_RATE, SYMBOL_RATE> clock_recovery;
    M17Framer<368> framer;
    M17FrameDecoder decoder;
    DemodState demodState = DemodState::UNLOCKED;
    M17FrameDecoder::SyncWordType sync_word_type = M17FrameDecoder::SyncWordType::LSF;
    uint8_t sample_index = 0;

    bool dcd_ = false;
    bool passall_ = false;
    int ber = -1;
    int16_t sync_count = 0;
    uint8_t missing_sync_count = 0;

    virtual ~M17Demodulator() {}

    void start() override;

    void dcd_on();
    void dcd_off();

    void stop() override
    {
//        getModulator().stop_loopback();
        stopADC();
        dcd_off();
    }

    bool locked() const override
    {
        return dcd_;
    }

    size_t size() const override
    {
        return ADC_BLOCK_SIZE;
    }

    void passall(bool enabled) override
    {
        passall_ = enabled;
        decoder.passall(enabled);
    }

    demod_result_t demod(float sample);

    void update_values(uint8_t index);

    hdlc::IoFrame* operator()(const q15_t* input) override;

    /*
     * Return twist as a the difference in dB between mark and space.  The
     * expected values are about 0dB for discriminator output and about 5.5dB
     * for de-emphasized audio.
     */
    float readTwist() override
    {
        return 0;
    }

    uint32_t readBatteryLevel() override
    {
#ifndef NUCLEOTNC
        TNC_DEBUG("enter M17Demodulator::readBatteryLevel");

        htim6.Init.Period = 48000;
        if (HAL_TIM_Base_Init(&htim6) != HAL_OK) CxxErrorHandler();

        if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
            CxxErrorHandler();

        // Disable battery charging while measuring battery voltage.
        auto usb_ce = gpio::USB_CE::get();
        gpio::USB_CE::on();

        gpio::BAT_DIVIDER::off();
        HAL_Delay(1);

        uint32_t vbat = 0;
        if (HAL_ADC_Start(&BATTERY_ADC_HANDLE) != HAL_OK) CxxErrorHandler();
        for (size_t i = 0; i != 8; ++i)
        {
            if (HAL_ADC_PollForConversion(&BATTERY_ADC_HANDLE, 1) != HAL_OK) CxxErrorHandler();
            vbat += HAL_ADC_GetValue(&BATTERY_ADC_HANDLE);
        }

        vbat /= 8;

        if (HAL_ADC_Stop(&BATTERY_ADC_HANDLE) != HAL_OK) CxxErrorHandler();
        if (HAL_TIM_Base_Stop(&htim6) != HAL_OK)
            CxxErrorHandler();

        gpio::BAT_DIVIDER::on();

        // Restore battery charging state.
        if (!usb_ce) gpio::USB_CE::off();

        INFO("Vbat = %lu (raw)", vbat);

        // Order of operations is important to avoid underflow.
        vbat *= 12375;
        vbat /= (VREF + 1);

        TNC_DEBUG("exit M17Demodulator::readBatteryLevel");
        return vbat;
#else
        return 0;
#endif
    }
};

}} // mobilinkd::tnc
