// Copyright 2017-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

void startIOEventTask(void* argument);

extern osMessageQueueId_t ioEventQueueHandle;
#ifndef NUCLEOTNC
extern volatile int cdc_connected;
#endif

#ifdef __cplusplus
}

namespace mobilinkd { namespace tnc {

void print_startup_banner() __attribute__((noinline));

}} // mobilinkd::tnc

#endif
