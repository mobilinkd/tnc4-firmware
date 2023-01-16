// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AFSKModulator.hpp"

namespace mobilinkd { namespace tnc {

void AFSKModulator::init(const kiss::Hardware& hw)
{
    set_twist(hw.tx_twist);

    // Configure 48MHz clock for 26.4ksps.
    htim7.Init.Period = 1817;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        ERROR("htim7 init failed");
        CxxErrorHandler();
    }
}

}} // mobilinkd::tnc
