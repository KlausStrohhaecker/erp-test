#include "erp/ERP.h"
#include "cmsis/LPC43xx.h"
#include "midi/MIDI_relay.h"

void ERP_Init(void)
{
#define ADC_SETUP                                                                               \
  (((3 - 1) << ADC0_CR_CLKDIV_Pos) /* 12MHz / 3 --> 4MHz */                                     \
   | (1 << ADC0_CR_BURST_Pos)      /* burst mode, continuous read */                            \
   | (0 << ADC0_CR_CLKS_Pos)       /* "11" clocks --> 10 bits resolution, 363kHz sample rate */ \
   | (1 << ADC0_CR_PDN_Pos)        /* enable ADC */                                             \
   | (0 << ADC0_CR_START_Pos))     /* start = 0 required for burst mode */
                                   // effective sample rate per ERP is 363kHz / 4 = 90.9kHz

  LPC_ADC0->CR = (0b11111111 << ADC0_CR_SEL_Pos)
      | ADC_SETUP;
  LPC_ADC1->CR = (0b11111111 << ADC0_CR_SEL_Pos)
      | ADC_SETUP;
}

static void process(void)
{
  static int32_t erpData[35] = { 2000, ERP_SCALE_FACTOR, 0 };  // sample rate, scale factor, packet-number, 32 values

  erpData[2]++;  // packet number

  for (int i = 0; i < 1; i++)
    erpData[i + 3] = ERP_DecodeWipersToAngle((LPC_ADC0->DR[i] & 0xFFFF) >> 6, (LPC_ADC0->DR[i + 4] & 0xFFFF) >> 6);

  static uint8_t sysexBuffer[sizeof(erpData) * 2];

  SendERP(erpData, sizeof(erpData), sysexBuffer);
}

void ERP_Process(void)
{
  static unsigned timeSlice = 48000;  // startup delay of 6 seconds
  if (!--timeSlice)
  {
    timeSlice = 4;
    process();
  }
}
