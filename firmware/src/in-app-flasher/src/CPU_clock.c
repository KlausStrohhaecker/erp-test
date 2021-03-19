#include <stdint.h>
#include "cmsis/lpc43xx_cgu.h"

uint32_t M4coreClock;

static void Delay300(void);

/******************************************************************************/
/** @brief    	configures the clocks of the whole system
	@details	The main cpu clock should set to 204 MHz. According to the
				datasheet, therefore the clock has to be increased in two steps.
				1. low frequency range to mid frequency range
				2. mid frequency range to high frequency range
				clock ranges:
				    - low		f_cpu: < 90 MHz
				    - mid		f_cpu: 90 MHz to 110 MHz
				  	- high		f_cpu: up to 204 MHz
				settings (f: frequency, v: value):
					- f_osc  =  12 MHz
					- f_pll1 = 204 MHz		v_pll1 = 17
					- f_cpu  = 204 MHz
*******************************************************************************/
void CPU_ConfigureClocks(void)
{
  /* XTAL OSC */
  CGU_SetXTALOSC(12000000);                                 // set f_osc = 12 MHz (external XTAL OSC is a 12 MHz device)
  Delay300();                                               // delay at least 300 µs
  CGU_EnableEntity(CGU_CLKSRC_XTAL_OSC, ENABLE);            // Enable xtal osc clock entity as clock source
  Delay300();                                               // delay at least 300 µs
  CGU_EntityConnect(CGU_CLKSRC_XTAL_OSC, CGU_CLKSRC_PLL1);  // connect XTAL to PLL1
  Delay300();                                               // delay at least 300 µs

  /* STEP 1: set cpu to mid frequency (according to datasheet) */
  CGU_SetPLL1(8);  // f_osc x 8 = 96 MHz
  M4coreClock = 8ul * 12000000ul;
  Delay300();                                 // delay at least 300 µs
  CGU_EnableEntity(CGU_CLKSRC_PLL1, ENABLE);  // Enable PLL1 after setting is done
  Delay300();                                 // delay at least 300 µs
  CGU_EntityConnect(CGU_CLKSRC_PLL1, CGU_BASE_M4);
  Delay300();  // delay at least 300 µs
  CGU_UpdateClock();
  Delay300();  // delay at least 300 µs

  /* STEP 2: set cpu to a high frequency */
  CGU_SetPLL1(17);  // set PLL1 to: f_osc x 17 = 204 MHz
  M4coreClock = 17ul * 12000000ul;
  Delay300();         // delay at least 300 µs
  CGU_UpdateClock();  // Update Clock Frequency
  Delay300();         // delay at least 300 µs
}

/******************************************************************************/
/** @brief  	modul internal delay function - waits at least 300 µs @ 200 MHz
				with lower frequencies it waits longer
*******************************************************************************/
static void Delay300(void)
{
  //  register uint32_t cnt = 60000;  // 60'000 * 5ns = 300us
  register uint32_t cnt = 15000;  // 15'000 * 20ns = 300us
  while (--cnt)
    asm volatile("nop");  // 1 cycle = 5ns (@200mHz)
}
