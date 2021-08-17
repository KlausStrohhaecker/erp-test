#define INT_MAX (0x7FFFFFFFu)

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "ERP.h"
#include "interpol.h"

// ----------------------------------------------------

#define ERP_AVERAGING_TIME_ms (100ll)  // 100ms floating window average
#define ERP_FILTER_TIME_CONST (0.5)    // velocity lowpass filter time constant (in seconds)

// fixed process constants
#define ERP_TICKS_PER_DEGREE (100ll)  // 0.01 degree/increment is finest possible range just at the noise threshold

#define ERP_INCR_SCALE_FACTOR (128)

typedef struct
{
  ERP_QuantizerInit_t initData;
  int64_t *           buffer;
  unsigned            bufSize;
  unsigned            bufIndex;
  int64_t             bufAverage;
  int64_t             summedAverage;
  int64_t             summedTicks;
  double              lambda;
  double              ticksSmoothed;
  int                 fill;
  int64_t             fineThreshold;
  int32_t             table_x[3];
  int32_t             table_y[3];
  LIB_interpol_data_T table;
} ERP_Quantizer_t;

void *ERP_InitQuantizer(const ERP_QuantizerInit_t initData)
{
  ERP_Quantizer_t *quantizer = malloc(sizeof(ERP_Quantizer_t));
  if (!quantizer)
    return (void *) 0;

  // buffer size = averaging time * sample rate
  quantizer->bufSize = ERP_AVERAGING_TIME_ms * initData.sampleRate / 1000u;
  quantizer->buffer  = calloc(quantizer->bufSize, sizeof(quantizer->buffer[0]));
  if (!quantizer->buffer)
  {
    free(quantizer);
    return (void *) 0;
  }
  quantizer->initData       = initData;
  quantizer->fill           = 2;
  quantizer->bufIndex       = 0;
  quantizer->lambda         = 1.0 - exp(-1 / ERP_FILTER_TIME_CONST / initData.sampleRate);
  quantizer->ticksSmoothed  = 0.0;
  quantizer->fineThreshold  = ERP_SCALE_FACTOR * quantizer->bufSize / 3600ll;
  quantizer->table_x[0]     = initData.velocityStart;
  quantizer->table_x[2]     = initData.velocityStop;
  quantizer->table_x[1]     = initData.velocityStart + (quantizer->table_x[2] - quantizer->table_x[0]) * initData.splitPointVelocity;
  quantizer->table_y[0]     = ERP_INCR_SCALE_FACTOR * initData.incrementsPerDegreeStart;
  quantizer->table_y[2]     = ERP_INCR_SCALE_FACTOR * initData.incrementsPerDegreeStop;
  quantizer->table_y[1]     = quantizer->table_y[0] + (quantizer->table_y[2] - quantizer->table_y[0]) * initData.splitPointIncrement;
  quantizer->table.points   = 3;
  quantizer->table.x_values = quantizer->table_x;
  quantizer->table.y_values = quantizer->table_y;
  return (void *) quantizer;
}

int ERP_ExitQuantizer(void *const quantizer)
{
  if (!quantizer)
    return 0;
  ERP_Quantizer_t *q = quantizer;
  if (q->buffer)
    free(q->buffer);
  free(quantizer);
  return 1;
}

static inline int64_t abs64(int64_t const x)
{
  return (x < 0) ? -x : x;
}

static inline uint64_t lookUpDynamicThreshold(ERP_Quantizer_t *const q, int const ticks)
{
  q->ticksSmoothed      = q->ticksSmoothed - (q->lambda * (q->ticksSmoothed - (double) abs(ticks)));  // number of 0.1deg ticks per 500us
  int      degPerSecond = (int) (q->initData.sampleRate * q->ticksSmoothed / ERP_TICKS_PER_DEGREE);   // * sample rate / ticks per degree ==> degrees per second
  uint64_t result       = ERP_INCR_SCALE_FACTOR * ERP_TICKS_PER_DEGREE * q->fineThreshold / LIB_InterpolateValue(&(q->table), degPerSecond);
  printf("\n%6d \033[1A", degPerSecond);
  return result;
}

int ERP_getDynamicIncrement(void *const quantizer, int const increment)
{
  ERP_Quantizer_t *const q = quantizer;

  q->bufIndex = (q->bufIndex + 1) % q->bufSize;
  q->bufAverage -= q->buffer[q->bufIndex];
  q->bufAverage += increment;
  q->buffer[q->bufIndex] = increment;

  if (q->fill)
  {
    if (q->bufIndex == 0)
      if (!--(q->fill))
        q->summedTicks = q->summedAverage = q->bufAverage;
    return 0;
  }

  // determine velocity
  q->summedTicks += q->bufAverage;
  int ticks = 0;
  if (abs64(q->summedTicks) > q->fineThreshold)
  {
    ticks = (int) q->summedTicks / q->fineThreshold;
    if (ticks >= 0)
      q->summedTicks = q->summedTicks % q->fineThreshold;
    else
      q->summedTicks = -(-q->summedTicks % q->fineThreshold);
  }

  long threshold = lookUpDynamicThreshold(q, ticks);

  q->summedAverage += q->bufAverage;
  if (abs64(q->summedAverage) > threshold)
  {
    int retval = (int) (q->summedAverage / threshold);
#if PRECISION_SNAPPOINT
    if (retval >= 0)
      q->summedAverage = q->summedAverage % threshold;
    else
      q->summedAverage = -(-q->summedAverage % threshold);
#else
    q->summedAverage = 0;
#endif
    return retval;
  }
  return 0;
}
