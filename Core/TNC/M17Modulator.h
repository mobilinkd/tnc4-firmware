// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "AudioInput.hpp"
#include "Modulator.hpp"
#include "HdlcFrame.hpp"
#include "M17.h"

#include <arm_math.h>

#include <array>
#include <algorithm>
#include <atomic>
#include <cstdint>

extern IWDG_HandleTypeDef hiwdg;

namespace mobilinkd { namespace tnc {

/**
 * M17 modulator. Collects 16 symbols of data, upsamples by 10x using
 * an interpolating FIR filter, which is sent out via the DAC using
 * DMA.
 */
struct M17Modulator : Modulator
{
    // Six buffers per M17 frame, or 12 half-buffer interrupts.
    static constexpr uint8_t UPSAMPLE = 10;
    static constexpr uint32_t BLOCKSIZE = 4;
    static constexpr uint32_t STATE_SIZE = (m17::FILTER_TAP_NUM / UPSAMPLE) + BLOCKSIZE - 1;
    // Number of bytes (4 symbol groups) to flush the FIR filter.
    static constexpr uint8_t FLUSH_LEN = ((m17::FILTER_TAP_NUM / UPSAMPLE) + 3) / 4;
    static constexpr int16_t DAC_BUFFER_LEN = 80;               // 8 symbols, 16 bits, 2 bytes.
    static constexpr int16_t TRANSFER_LEN = DAC_BUFFER_LEN / 2; // 4 symbols, 8 bits, 1 byte.
    static constexpr uint16_t VREF = 4095;
    enum class State { STOPPED, STARTING, RUNNING, STOPPING };

    arm_fir_interpolate_instance_f32 fir_interpolator;
    std::array<float, STATE_SIZE> fir_state;
    std::array<int16_t, DAC_BUFFER_LEN> buffer_;
    std::array<float, 4> symbols;
    osMessageQId dacOutputQueueHandle_{0};
    PTT* ptt_{nullptr};
    uint16_t volume_{4096};
    std::atomic<uint16_t> delay_count = 0;      // TX Delay
    std::atomic<uint16_t> stop_count = 0;       // Flush the RRC matched filter.
    State state{State::STOPPED};
    float tmp[TRANSFER_LEN];
    bool send_tone = false;

    M17Modulator(osMessageQId queue, PTT* ptt)
    : dacOutputQueueHandle_(queue), ptt_(ptt)
    {
        arm_fir_interpolate_init_f32(
            &fir_interpolator, UPSAMPLE, m17::FILTER_TAP_NUM,
            (float32_t*) m17::rrc_taps_f.data(), fir_state.data(), BLOCKSIZE);
    }

    ~M17Modulator() override {}

    void start_loopback() override
    {
    }

    void stop_loopback() override
    {
    }

    void loopback(const void* input) override
    {
    }

    void init(const kiss::Hardware& hw) override;

    void deinit() override
    {
        state = State::STOPPED;
        HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop(&htim7);
        ptt_->off();
    }

    void set_gain(uint16_t level) override
    {
        auto v = std::max<uint16_t>(256, level);
        v = std::min<uint16_t>(4096, v);
        volume_ = v;
    }

    void set_ptt(PTT* ptt) override
    {
        if (state != State::STOPPED)
        {
            ERROR("PTT change while not stopped");
            CxxErrorHandler();
        }
        ptt_ = ptt;
        ptt_->off();
    }

    PTT* get_ptt() const { return ptt_; }

    void send(uint8_t bits) override
    {
        uint16_t txdelay = 0;

        switch (state)
        {
        case State::STOPPING:
        case State::STOPPED:
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
            HAL_RCCEx_DisableLSCO();
#endif
            delay_count = 0;
            txdelay = kiss::settings().txdelay * 12 - 5;
            fill_empty(buffer_.data());
            fill_empty(buffer_.data() + TRANSFER_LEN);
            state = State::STARTING;
            [[fallthrough]];
        case State::STARTING:
            osMessagePut(audioInputQueueHandle, tnc::audio::IDLE,
              osWaitForever);
            start_conversion();
            ptt_->on();
            while (delay_count < txdelay) osThreadYield();
            stop_count = FLUSH_LEN;
            osMessagePut(dacOutputQueueHandle_, bits, osWaitForever);
            state = State::RUNNING;
            break;
        case State::RUNNING:
            osMessagePut(dacOutputQueueHandle_, bits, osWaitForever);
            break;
        }
    }

    constexpr std::array<float, 48> make_1000hz_tone()
    {
        std::array<float, 48> result;
        for (size_t i = 0; i != result.size(); ++i) {
            result[i] = std::sin(M_PI * i * 2.0 / result.size()) * 3;
        }
        return result;
    }

    void tone(uint16_t) override
    {
        send_tone = true;
    }

    // DAC DMA interrupt functions.
    [[gnu::noinline]]
    void fill_first(uint8_t bits) override
    {
        fill(buffer_.data(), bits);
    }

    [[gnu::noinline]]
    void fill_last(uint8_t bits) override
    {
        fill(buffer_.data() + TRANSFER_LEN, bits);
    }

    /*
     * DAC queue is empty when STARTING.  It is filled with '0' symbols
     * for TX delay duration (using delay_count).  It then transitions
     * to the running state in send() after filling the DAC queue.
     *
     * When no more symbols are available, the DAC queue is empty in the
     * running state.  The FIR filter is flushed of the remaining data
     * using '0' symbols.
     */
    void empty_first() override
    {
        switch (state)
        {
        case State::STARTING:
            fill_empty(buffer_.data());
            delay_count += 1;
            break;
        case State::RUNNING:
            fill_empty(buffer_.data());
            state = State::STOPPING;
            break;
        case State::STOPPING:
            fill_empty(buffer_.data());
            if (--stop_count == 0) state = State::STOPPED;
            break;
        case State::STOPPED:
            stop_conversion();
            ptt_->off();
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
                HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
            osMessagePut(audioInputQueueHandle, tnc::audio::DEMODULATOR,
              osWaitForever);

            break;
        }
    }

    /*
     * DAC queue is empty when STARTING.  It is filled with '0' symbols
     * for TX delay duration (using delay_count).  It then transitions
     * to the running state in send() after filling the DAC queue.
     *
     * When no more symbols are available, the DAC queue is empty in the
     * running state.  The FIR filter is flushed of the remaining data
     * using '0' symbols.
     */
    void empty_last() override
    {
        switch (state)
        {
        case State::STARTING:
            fill_empty(buffer_.data() + TRANSFER_LEN);
            delay_count += 1;
            break;
        case State::RUNNING:
            fill_empty(buffer_.data() + TRANSFER_LEN);
            state = State::STOPPING;
            break;
        case State::STOPPING:
            fill_empty(buffer_.data() + TRANSFER_LEN);
            if (--stop_count == 0) state = State::STOPPED;
            break;
        case State::STOPPED:
            stop_conversion();
            ptt_->off();
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
                HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
            osMessagePut(audioInputQueueHandle, tnc::audio::DEMODULATOR,
              osWaitForever);
            break;
        }
    }

    void abort() override
    {
        state = State::STOPPED;
        send_tone = false;
        stop_conversion();
        ptt_->off();
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
            HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
        // Drain the queue.
        while (osMessageGet(dacOutputQueueHandle_, 0).status == osEventMessage);
    }

    float bits_per_ms() const override
    {
        return 9.6f;
    }

private:

    /**
     * Configure the DAC for timer-based DMA conversion, start the timer,
     * and start DMA to DAC.
     */
    void start_conversion()
    {
        DAC_ChannelConfTypeDef sConfig;

        sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
        sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
        sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
        sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
        sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
        if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
        {
          CxxErrorHandler();
        }

        HAL_TIM_Base_Start(&htim7);
        HAL_DAC_Start_DMA(
            &hdac1, DAC_CHANNEL_1,
            reinterpret_cast<uint32_t*>(buffer_.data()), buffer_.size(),
            DAC_ALIGN_12B_R);
    }

    uint16_t adjust_level(float sample) const
    {
        sample *= volume_;
        sample /= 8;
        sample += 2048;
        if (sample > 4095) sample = 4095;
        else if (sample < 0) sample = 0;
        return sample;
    }

    constexpr int8_t bits_to_symbol(uint8_t bits)
    {
        switch (bits)
        {
        case 0: return 1;
        case 1: return 3;
        case 2: return -1;
        case 3: return -3;
        }
        return 0;
    }

    constexpr int16_t symbol_skew(int32_t symbol)
    {
        const size_t shift = 8;
        if (symbol == 1 || symbol == -1)
        {
            int32_t offset = ((symbol * (kiss::settings().tx_twist - 50)) << shift) / 50;
            return (symbol << shift) - offset;
        }
        else
        {
            return symbol << shift;
        }
    }

    void fill_tone(int16_t* buffer)
    {
        static uint8_t pos = 0;
        static const auto Hz1000 = make_1000hz_tone();

        int16_t polarity = kiss::settings().tx_rev_polarity() ? -1 : 1;

        for (size_t i = 0; i != TRANSFER_LEN; ++i) {
            buffer[i] = adjust_level(Hz1000[pos++] * polarity);
            if (pos == Hz1000.size()) pos = 0;
        }
    }

    [[gnu::noinline]]
    void fill(int16_t* buffer, uint8_t bits)
    {
        HAL_IWDG_Refresh(&hiwdg);

        if (send_tone)
        {
            fill_tone(buffer);
            return;
        }

        int16_t polarity = kiss::settings().tx_rev_polarity() ? -1 : 1;

        for (size_t i = 0; i != 4; ++i)
        {
            symbols[i] = bits_to_symbol(bits >> 6) * polarity;
            bits <<= 2;
        }

        arm_fir_interpolate_f32(
            &fir_interpolator, symbols.data(), tmp, BLOCKSIZE);

        for (size_t i = 0; i != TRANSFER_LEN; ++i)
        {
            buffer[i] = adjust_level(tmp[i]);
        }
    }
#if 0
    [[gnu::noinline]]
    void fill_empty(int16_t* buffer)
    {
        symbols.fill(0);

        arm_fir_interpolate_f32(
            &fir_interpolator, symbols.data(), tmp, BLOCKSIZE);

        for (size_t i = 0; i != TRANSFER_LEN; ++i)
        {
            buffer[i] = adjust_level(tmp[i]);
        }
    }
#endif

    [[gnu::noinline]]
    void fill_empty(int16_t* buffer)
    {
        HAL_IWDG_Refresh(&hiwdg);
        send_tone = false;
        for (size_t i = 0; i != TRANSFER_LEN; ++i)
        {
            buffer[i] = 2048;
        }
    }
};

}} // mobilinkd::tnc
