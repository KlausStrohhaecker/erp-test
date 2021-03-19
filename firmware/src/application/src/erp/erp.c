#include "erp.h"
#include "cmsis/LPC43xx.h"
#include "midi/nl_sysex.h"
#include "midi/MIDI_relay.h"

void ERP_Init(void)
{
#define ADC_SETUP                                                                           \
  ((51 << ADC0_CR_CLKDIV_Pos)  /* 204MHz / 51 --> 4MHz */                                   \
   | (1 << ADC0_CR_BURST_Pos)  /* burst mode, continuous read */                            \
   | (0 << ADC0_CR_CLKS_Pos)   /* "11" clocks --> 10 bits resolution, 363kHz sample rate */ \
   | (1 << ADC0_CR_PDN_Pos)    /* enable ADC */                                             \
   | (0 << ADC0_CR_START_Pos)) /* start = 0 required for burst mode */
                               // effective sample rate per ERP is 363kHz / 4 = 90.9kHz

  LPC_ADC0->CR = (0b00001111 << ADC0_CR_SEL_Pos)  // ADC0 scans Wiper 1 (@lane 0..3) of all ERPs
      | ADC_SETUP;
  LPC_ADC1->CR = (0b11110000 << ADC0_CR_SEL_Pos)  // ADC1 scans Wiper 2 (@lane 4..7) of all ERPs
      | ADC_SETUP;
}

static void process(void)
{
  static uint16_t adcValues[8];  // 4x Wiper 1, 4x Wiper 2

  adcValues[0] = LPC_ADC0->DR[0] >> 6;
  adcValues[1] = LPC_ADC0->DR[1] >> 6;
  adcValues[2] = LPC_ADC0->DR[2] >> 6;
  adcValues[3] = LPC_ADC0->DR[3] >> 6;

  adcValues[4] = LPC_ADC1->DR[4] >> 6;
  adcValues[5] = LPC_ADC1->DR[5] >> 6;
  adcValues[6] = LPC_ADC1->DR[6] >> 6;
  adcValues[7] = LPC_ADC1->DR[7] >> 6;

  static uint8_t sysexBuffer[64];

  uint16_t size = MIDI_encodeRawSysex((uint8_t *) &adcValues[0], 16, sysexBuffer);
  SendERP(sysexBuffer, size);
}

void ERP_Process(void)
{
  static unsigned timeSlice = 24000;
  if (!--timeSlice)
  {
    timeSlice = 4;
    process();
  }
}
