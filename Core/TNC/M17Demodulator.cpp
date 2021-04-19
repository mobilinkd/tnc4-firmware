// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "M17Demodulator.h"

#include "main.h"

#include "stm32l4xx_hal.h"

#include <array>
#include <cstdint>

namespace mobilinkd { namespace tnc {

void M17Demodulator::start()
{
    SysClock72();

    demod_filter.init(m17::rrc_taps_11.data());
    passall(kiss::settings().options & KISS_OPTION_PASSALL);

    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        CxxErrorHandler();
    }

    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = AUDIO_IN;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        CxxErrorHandler();

    startADC(1499, ADC_BLOCK_SIZE);
//    getModulator().start_loopback();
}

constexpr int8_t SAMPLES_PER_SYNC_WORD = 80;

hdlc::IoFrame* M17Demodulator::operator()(const q15_t* input)
{
    hdlc::IoFrame* result = nullptr;

    auto filtered = demod_filter.filter(const_cast<q15_t*>(input));
//    getModulator().loopback(filtered);

    if (demodState == DemodState::LSF_SYNC)
    {
        for (size_t i = 0; i != ADC_BLOCK_SIZE; ++i)
        {
            conv_buffer[conv_buffer_count++] = filtered[i];
            if (conv_buffer_count == SAMPLES_PER_SYNC_WORD)
            {
                conv_buffer_count = 0;
            }
        }
    }

    for (size_t i = 0; i != ADC_BLOCK_SIZE; ++i)
    {
        auto filtered_sample = filtered[i];

        if (sample_now)
        {
            samples[2] = filtered_sample;
            sample_now = false;
            frame(demod(), result);
        }
        else
        {
            t += dt;
            if (t < 1.0)
            {
                samples[0] = filtered_sample;
            }
            else
            {
                t -= 1.0;
                samples[1] = filtered_sample;
                sample_now = true;
            }
        }
    }
    return result;
}

}} // mobilinkd::tnc
