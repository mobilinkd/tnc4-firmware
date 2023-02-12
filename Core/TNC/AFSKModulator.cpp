// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AFSKModulator.hpp"

namespace mobilinkd { namespace tnc {

void AFSKModulator::init(const kiss::Hardware &hw)
{
    set_twist(hw.tx_twist);

    // Configure 48MHz clock for 26.4ksps.
    htim7.Instance->ARR = 1817;
    htim7.Instance->PSC = 0;

    DAC_ChannelConfTypeDef sConfig;

    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        CxxErrorHandler();
    }

    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048) != HAL_OK)
        CxxErrorHandler();
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
        CxxErrorHandler();
}

}} // mobilinkd::tnc
