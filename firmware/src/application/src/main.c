/******************************************************************************/
/** @file	Emphase_M4_Main.c
    @date	2016-03-09 SSC
	@changes 2020-05-11 KSTR
*******************************************************************************/

#pragma GCC diagnostic ignored "-Wmain"

#include <stdint.h>

#include "CPU_clock.h"
#include "drv/nl_leds.h"
#include "drv/nl_cgu.h"
#include "io/pins.h"
#include "midi/MIDI_relay.h"
#include "midi/MIDI_statemonitor.h"
#include "sys/nl_version.h"
#include "sys/ticker.h"

void M4SysTick_Init(void);

/******************************************************************************/
static int periodicTimer = TIME_SLICE;
static int trigger       = 0;

volatile char dummy;
void          dummyFunction(const char *string)
{
  while (*string)
    dummy = *string++;
}

void main(void)
{
  // referencing the version string so compiler won't optimize it away
  dummyFunction(VERSION_STRING);

  CPU_ConfigureClocks();
  PINS_Init();
  M4SysTick_Init();
  // we wait until here because USB handlers are interrupt-driven and everything has to be set up
  MIDI_Relay_Init();

  while (1)
  {
    MIDI_Relay_ProcessFast();
    if (trigger)
    {
      trigger = 0;
      SMON_Process();
    }
  }
}

/*************************************************************************/ /**
* @brief	ticker interrupt routines using the standard M4 system ticker
*           IRQ will be triggered every 125us, our basic scheduler time-slice
******************************************************************************/
void M4SysTick_Init(void)
{
#define SYST_CSR   (uint32_t *) (0xE000E010)  // SysTick Control & Status Reg
#define SYST_RVR   (uint32_t *) (0xE000E014)  // SysTick Reload Value
#define SYST_CVR   (uint32_t *) (0xE000E018)  // SysTick Counter Value
#define SYST_CALIB (uint32_t *) (0xE000E01C)  // SysTick Calibration
  *SYST_RVR = (M4coreClock / M4_FREQ_HZ) - 1;
  *SYST_CVR = 0;
  *SYST_CSR = 0b111;  // processor clock | IRQ enabled | counter enabled
}

// clock tick handler (relies on the exact name of it!)
void SysTick_Handler(void)
{
  ticker++;
  if (!--periodicTimer)
  {
    periodicTimer = TIME_SLICE;
    trigger       = 1;
  }
}
