// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "cmsis_os2.h"

extern "C" void startAfskToneTask(void* arg);

namespace mobilinkd { namespace tnc {

struct AFSKTestTone
{
    enum class State {MARK, SPACE, BOTH, NONE};
    void transmit(State prev);
    void mark();
    void space();
    void both();
    void stop();
    void fill() const;
    State state() const { return state_; }

    State state_{State::NONE};
    osThreadId_t testToneTask_{0};
};

}} // mobilinkd::tnc
