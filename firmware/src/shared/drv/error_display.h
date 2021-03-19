/******************************************************************************/
/** @file		error_display.h
*******************************************************************************/
#pragma once

typedef enum
{
  E_NONE,
  E_CODE_ERROR,
  E_USB_PACKET_SIZE,
  E_USB_UNEXPECTED_PACKET,
  E_SYSEX_DATA,
  E_SYSEX_INCOMPLETE,
  E_PROG_UPDATE_TOO_LARGE,
  E_PROG_ZERO_DATA,
  E_PROG_ERASE,
  E_PROG_WRITEPREPARE,
  E_PROG_WRITE,
  E_PROG_SUCCESS,
} ErrorEvent_t;
#define ErrorEvent_t_Size (E_PROG_SUCCESS + 1)

void DisplayError(ErrorEvent_t const err);         // just turn on the LEDs (no blinking)
void DisplayErrorAndHalt(ErrorEvent_t const err);  // enter the corresponding blink pattern, if any
