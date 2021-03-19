/******************************************************************************/
/** @file		error_display.c
*******************************************************************************/
#include "drv/error_display.h"
#include "drv/nl_leds.h"

static uint8_t RGB_LED[2];
// clang-format off
static uint8_t const LED_PATTERNS[ErrorEvent_t_Size][2] =
{
  [E_NONE]                  = { COLOR_OFF    , COLOR_OFF },  // this one is non-blocking
  [E_CODE_ERROR]            = { COLOR_WHITE  , COLOR_WHITE },
  [E_USB_PACKET_SIZE]       = { COLOR_RED    , COLOR_RED     + LED_BLINKING },
  [E_USB_UNEXPECTED_PACKET] = { COLOR_RED    , COLOR_YELLOW  + LED_BLINKING },
  [E_SYSEX_DATA]            = { COLOR_YELLOW , COLOR_RED     + LED_BLINKING },
  [E_SYSEX_INCOMPLETE]      = { COLOR_YELLOW , COLOR_YELLOW },  // this one is non-blocking
  [E_PROG_UPDATE_TOO_LARGE] = { COLOR_MAGENTA, COLOR_RED     + LED_BLINKING },
  [E_PROG_ZERO_DATA]        = { COLOR_MAGENTA, COLOR_GREEN   + LED_BLINKING },
  [E_PROG_ERASE]            = { COLOR_MAGENTA, COLOR_BLUE    + LED_BLINKING },
  [E_PROG_WRITEPREPARE]     = { COLOR_MAGENTA, COLOR_MAGENTA + LED_BLINKING },
  [E_PROG_WRITE]            = { COLOR_MAGENTA, COLOR_WHITE   + LED_BLINKING },
  [E_PROG_SUCCESS]          = { COLOR_GREEN + LED_BLINKING, COLOR_GREEN + LED_BLINKING },
};
// clang-format on

void DisplayError(ErrorEvent_t const err)
{
  RGB_LED[0] = LED_PATTERNS[err][0];
  LED_SetDirect(0, RGB_LED[0]);
  RGB_LED[1] = LED_PATTERNS[err][1];
  LED_SetDirect(1, RGB_LED[1]);
}

void DisplayErrorAndHalt(ErrorEvent_t const err)
{
  __disable_irq();
  DisplayError(err);

#define MAX (3000000ul);
  int clear = 0;
  do
  {
    uint32_t cntr = MAX;
    while (cntr--)
      asm volatile("nop");

    DisplayError(err);

    clear = !clear;
    if (clear)
    {
      if (RGB_LED[0] & LED_BLINKING)
        LED_SetDirect(0, 0);
      if (RGB_LED[1] & LED_BLINKING)
        LED_SetDirect(1, 0);
    }
  } while (1);
}
