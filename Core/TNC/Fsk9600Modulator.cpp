// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Fsk9600Modulator.hpp"

namespace mobilinkd { namespace tnc {

/*
 * Cosine.
const Fsk9600Modulator::cos_table_type Fsk9600Modulator::cos_table = {
    2047,  2020,  1937,  1801,  1616,  1387,  1120,   822,   502,   169,
    -169,  -502,  -822, -1120, -1387, -1616, -1801, -1937, -2020, -2048
};
*/

/*
 * Square wave -- filtered in hardware at 7200Hz
const Fsk9600Modulator::cos_table_type Fsk9600Modulator::cos_table = {
     2047,  2047,  2047,  2047,  2047,  2047,  2047,  2047,  2047,  2047,
    -2048, -2048, -2048, -2048, -2048, -2048, -2048, -2048, -2048, -2048
};
*/

// Gaussian
const Fsk9600Modulator::cos_table_type Fsk9600Modulator::cos_table = {
    2042,  2027,  1995,  1931,  1815,  1626,  1345,   968,   507,     0,
    -507,  -968, -1345, -1626, -1815, -1931, -1995, -2027, -2042, -2048
};

void Fsk9600Modulator::init(const kiss::Hardware& hw)
{
    for (auto& x : buffer_) x = 2048;

    (void) hw; // unused

    state = State::STOPPED;
    level = Level::HIGH;

    SysClock72();

    // Configure 72MHz clock for 192ksps.
    htim7.Instance->ARR = 374; // 374
    htim7.Instance->PSC = 0;

    DAC_ChannelConfTypeDef sConfig;

    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
      CxxErrorHandler();
    }

    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048) != HAL_OK) CxxErrorHandler();
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) CxxErrorHandler();
    INFO("Fsk9600Modulator::init");
}

void Fsk9600Modulator::fill(uint16_t* buffer, bool bit)
{
    switch (level)
    {
    case Level::HIGH:
        if (bit)
        {
            std::fill(buffer, buffer + BIT_LEN, adjust_level(2047));
        }
        else
        {
            std::transform(cos_table.begin(), cos_table.end(), buffer,
                [this](auto x){return adjust_level(x);});
            level = Level::LOW;
        }
        break;
    case Level::LOW:
        if (bit)
        {
            std::transform(cos_table.begin(), cos_table.end(), buffer,
                [this](auto x){return adjust_level(-1 - x);});
            level = Level::HIGH;
        }
        else
        {
            std::fill(buffer, buffer + BIT_LEN, adjust_level(-2048));
        }
        break;
    default:
        CxxErrorHandler();
    }
}

}} // mobilinkd::tnc
