#pragma once

#include <stdint.h>

#define TIME_SLICE (1)  // time-slice for periodic tasks in M4_PERIOD_US (==125) usecs mutiples

extern volatile uint64_t ticker;
