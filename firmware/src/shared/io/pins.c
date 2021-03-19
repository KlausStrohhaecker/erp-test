#include "pin_setup.h"
#include "pins.h"

#ifdef EVAL_BOARD
#warning Forcing Compile for Evaluation Board specifics (GPIOs for LEDs etc)
#endif

uint8_t  isEvalPCB;
uint32_t __dummy_pin__;

// clang-format off

//
// --------------- LEDs ----------------

// System RGB-LEDs (those are active-low!)
void debugPinsInit(void)
{
  LED_A_init();
  LED_B_init();
  LED_C_init();
  LED_D_init();
  LED_E_init();
  LED_F_init();
  LED_RED0   = 0;
  LED_RED1   = 0;
  LED_GREEN0 = 0;
  LED_GREEN1 = 0;
  LED_BLUE0  = 0;
  LED_BLUE1  = 0;
  LED_DBG2_init();
  LED_DBG3_init();
  LED_DBG2 = 0;
  LED_DBG3 = 0;
}

// --------------- USB VBUS-Monitors ----------------

//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wunused-function"
int      pinUSB0_VBUS(void)
{
  if (isEvalPCB)
    return GPIO_Word(3, 0);
  else
    return GPIO_Word(2, 0);
}

int pinUSB1_VBUS(void)
{
  if (isEvalPCB)
    return GPIO_Word(3, 1);
  else
    return GPIO_Word(3, 0);
}

static void Delay300(void)
{
  register uint32_t cnt = 60000;  // 60'000 * 5ns = 300us
  while (--cnt)
    asm volatile("nop");  // 1 cycle = 5ns (@200mHz)
}
//#pragma GCC diagnostic pop

static inline void USBPinsInit(void)
{
  // GPIO 3[0] is USB0 for eval board ...
  // ... but also  USB1 for production board
  GPIO_DIR_IN(3, 0);
  SFSP(6, 1) = SFS_EIF + SFS_EIB + SFS_DHS + SFS_EPD + SFS_EPU + 0;

  // GPIO 3[1] is USB1 for eval board
  GPIO_DIR_IN(3, 1);
  SFSP(6, 2) = SFS_EIF + SFS_EIB + SFS_DHS + SFS_EPD + SFS_EPU + 0;

#ifndef EVAL_BOARD

  // Find board type (unless overridden)

  // Step one, sense pin with pullups enabled
  // GPIO 2[0] is USB0 for production board
  GPIO_DIR_IN(2, 0);
  SFSP(4, 0) = SFS_EIF + SFS_EIB + SFS_DHS + SFS_DPD + SFS_EPU + 0;
  Delay300();  // .3ms @204MHz
  uint32_t USB0_prod_PU = GPIO_Word(2, 0);

  // Step, 2 pulldowns enabled
  SFSP(4, 0) = SFS_EIF + SFS_EIB + SFS_DHS + SFS_EPD + SFS_DPU + 0;
  Delay300();  // .3ms @204MHz
  uint32_t USB0_prod_PD = GPIO_Word(2, 0);

  if ((USB0_prod_PU != 0) && (USB0_prod_PD == 0))
    isEvalPCB = 1;  // pin followed pullup/-down, so it is floating actually

  // set up follower/hysteresis behavior
  SFSP(4, 0) = SFS_EIF + SFS_EIB + SFS_DHS + SFS_EPD + SFS_EPD + 0;
  Delay300();  // .3ms @204MHz

#else

  isEvalPCB = 1;

#endif
}

// ------- Init all the pins -------
void PINS_Init(void)
{
  debugPinsInit();
  USBPinsInit();
}
