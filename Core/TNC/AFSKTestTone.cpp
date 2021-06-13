// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AFSKTestTone.hpp"
#include "ModulatorTask.hpp"
#include "Modulator.hpp"
#include "KissHardware.hpp"
#include "Log.h"

#include "stm32l4xx_hal.h"
#include "cmsis_os2.h"

extern RNG_HandleTypeDef hrng;
extern osThreadId testToneTaskHandle;

mobilinkd::tnc::AFSKTestTone mobilinkd::tnc::testTone;
void* testTonePtr = &mobilinkd::tnc::testTone;

void startAfskToneTask(void* arg)
{
    using mobilinkd::tnc::AFSKTestTone;

    INFO("startAfskToneTask")

    auto test = static_cast<AFSKTestTone*>(arg);

    while (true) {
        switch (test->state()) {
        case AFSKTestTone::State::NONE:
            osThreadYield();
            break;
        case AFSKTestTone::State::MARK:
        case AFSKTestTone::State::SPACE:
        case AFSKTestTone::State::BOTH:
            test->fill();
            break;
        default:
            break;
        }
    }
}

namespace mobilinkd { namespace tnc {

void AFSKTestTone::transmit(State prev)
{
    if (prev == State::NONE) {
      osThreadResume(testToneTaskHandle);
      INFO("RNG: CR = %08x, DR = %08x, SR = %08x", hrng.Instance->CR, hrng.Instance->DR, hrng.Instance->SR);
    }
}

void AFSKTestTone::mark()
{
    auto prev = state_;
    state_ = State::MARK;
    transmit(prev);
}

void AFSKTestTone::space()
{
    auto prev = state_;
    state_ = State::SPACE;
    transmit(prev);
}

void AFSKTestTone::both()
{
    auto prev = state_;
    state_ = State::BOTH;
    transmit(prev);
}

void AFSKTestTone::stop()
{
    if (state_ == State::NONE) return;

    state_ = State::NONE;
    getModulator().abort();
    osThreadSuspend(testToneTaskHandle);
}

void AFSKTestTone::fill()
{
	while (lfsr_ == 0) lfsr_ = HAL_GetTick() & 0x00ffffff;

    switch (state_) {
    case AFSKTestTone::State::NONE:
        return;
    case AFSKTestTone::State::MARK:
        if (kiss::settings().modem_type == kiss::Hardware::ModemType::M17)
        {
            getModulator().send(0x77);
        }
        else
        {
            getModulator().send(true);
        }
        break;
    case AFSKTestTone::State::SPACE:
        if (kiss::settings().modem_type == kiss::Hardware::ModemType::M17)
        {
            getModulator().send(0x22);
        }
        else
        {
            getModulator().send(false);
        }
        break;
    case AFSKTestTone::State::BOTH:
        if (kiss::settings().modem_type == kiss::Hardware::ModemType::M17)
        {
        	/*
            if ((counter_ & 3) == 0)
            {
                auto status = HAL_RNG_GenerateRandomNumber(&hrng, &random_);
                if (status != HAL_OK)
                {
                    WARN("RNG failure code %d - %lu (%lu)", status, hrng.ErrorCode, counter_)
                }
            }
            getModulator().send(random_ & 0xFF);
            random_ >>= 8;
            counter_ += 1;
            */
        	getModulator().send(lfsr_ & 0xFF);
        	for (int i = 0; i != 8; ++i)
        		lfsr_ = ((__builtin_popcount(lfsr_ & 0x87lu) & 1) << 23) | (lfsr_ >> 1);
        }
        else
        {
            getModulator().send(current_state_ == State::SPACE);
            current_state_ = (current_state_ == State::MARK ? State::SPACE : State::MARK);
        }
        break;
    default:
        break;
    }
}

}} // mobilinkd::tnc
