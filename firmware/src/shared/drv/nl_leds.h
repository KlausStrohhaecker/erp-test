/******************************************************************************/
/** @file		nl_leds.h
*******************************************************************************/
#pragma once

#include <stdint.h>
#include "io/pins.h"

typedef enum
{
  COLOR_OFF     = 0b0000,
  COLOR_RED     = 0b0001,
  COLOR_GREEN   = 0b0010,
  COLOR_BLUE    = 0b0100,
  COLOR_YELLOW  = 0b0011,
  COLOR_CYAN    = 0b0110,
  COLOR_MAGENTA = 0b0101,
  COLOR_WHITE   = 0b0111,
  LED_BLINKING  = 0b1000,
} LedColor_t;

static inline void LED_SetDirect(uint8_t const ledId, LedColor_t const rgb)
{
  if (ledId)
  {
    LED_RED1   = !(rgb & 0b001);
    LED_GREEN1 = !(rgb & 0b010);
    LED_BLUE1  = !(rgb & 0b100);
  }
  else
  {
    LED_RED0   = !(rgb & 0b001);
    LED_GREEN0 = !(rgb & 0b010);
    LED_BLUE0  = !(rgb & 0b100);
  }
}

static inline void LED_SetDirectAndHalt(uint8_t const rgb)
{
  LED_SetDirect(0, rgb);
  LED_SetDirect(1, rgb);
  __disable_irq();
  while (1)
    ;
}
