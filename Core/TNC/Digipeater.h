// Copyright 2026 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__DIGIPEATER_H_
#define MOBILINKD__TNC__DIGIPEATER_H_

#include "cmsis_os.h"

extern osThreadId digipeaterTaskHandle;
extern osMessageQId digipeaterQueueHandle;

void startDigipeaterTask(void* arg);
void beacon(void* arg);

#endif // MOBILINKD__TNC__DIGIPEATER_H_
