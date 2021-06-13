// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "cmsis_os2.h"

extern "C" void startAfskToneTask(void* arg);
extern "C" void* testTonePtr;

namespace mobilinkd { namespace tnc {

struct AFSKTestTone
{
    enum class State {MARK, SPACE, BOTH, NONE};
    void transmit(State prev);
    void mark();
    void space();
    void both();
    void stop();
    void fill();
    State state() const { return state_; }

    State state_ = State::NONE;
    State current_state_ = State::SPACE;
    uint32_t random_ = 0;
    uint32_t counter_ = 0;
    uint32_t lfsr_ = 0;
};

extern AFSKTestTone testTone;

}} // mobilinkd::tnc
