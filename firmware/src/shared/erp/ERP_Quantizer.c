#define INT_MAX (0x7FFFFFFFu)

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "ERP.h"
#include "interpol.h"

// ----------------------------------------------------

#define PRECISION_SNAPPOINT (0)  // set to !=0 to enable precision snap points rather than maximised hysteresis

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
  double              bufAverageSmoothed;
  int64_t             summedAverage;
  int64_t             summedTicks;
  double              lambda;
  double              workingLambda;
  double              ticksSmoothed;
  int                 fill;
  int64_t             fineThreshold;
  int32_t             table_x[4];
  int32_t             table_y[4];
  LIB_interpol_data_T table;
  uint16_t            activity;
  uint16_t            activityTimer;
} ERP_Quantizer_t;

void *ERP_InitQuantizer(const ERP_QuantizerInit_t initData)
{
  ERP_Quantizer_t *quantizer = calloc(sizeof(ERP_Quantizer_t), 1);
  if (!quantizer)
    return (void *) 0;

  // buffer size = averaging time * sample rate
  quantizer->bufSize = initData.slidingWindowTime * initData.sampleRate;
  quantizer->buffer  = calloc(quantizer->bufSize, sizeof(quantizer->buffer[0]));
  if (!quantizer->buffer)
  {
    free(quantizer);
    return (void *) 0;
  }
  quantizer->initData = initData;
  quantizer->fill     = 2;
  quantizer->bufIndex = 0;
  float timeConst     = initData.filterTimeConst;
  if (initData.adaptiveFiltering)
    timeConst *= 10.0;
  quantizer->lambda         = 1.0 - exp(-1 / timeConst / initData.sampleRate);
  quantizer->workingLambda  = quantizer->lambda;
  quantizer->ticksSmoothed  = 0.0;
  quantizer->fineThreshold  = ERP_SCALE_FACTOR * quantizer->bufSize / 3600ll;
  quantizer->table_x[0]     = initData.velocityStart;
  quantizer->table_x[3]     = initData.velocityStop;
  quantizer->table_x[1]     = initData.velocityStart + (quantizer->table_x[2] - quantizer->table_x[0]) * initData.splitPointVelocity[0];
  quantizer->table_x[2]     = initData.velocityStart + (quantizer->table_x[2] - quantizer->table_x[0]) * initData.splitPointVelocity[1];
  quantizer->table_y[0]     = ERP_INCR_SCALE_FACTOR * initData.incrementsPerDegreeStart;
  quantizer->table_y[3]     = ERP_INCR_SCALE_FACTOR * initData.incrementsPerDegreeStop;
  quantizer->table_y[1]     = quantizer->table_y[0] + (quantizer->table_y[2] - quantizer->table_y[0]) * initData.splitPointIncrement[0];
  quantizer->table_y[2]     = quantizer->table_y[0] + (quantizer->table_y[2] - quantizer->table_y[0]) * initData.splitPointIncrement[1];
  quantizer->table.points   = 4;
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
  q->ticksSmoothed      = q->ticksSmoothed - (q->workingLambda * (q->ticksSmoothed - (double) abs(ticks)));  // number of 0.1deg ticks per 500us
  int      degPerSecond = (int) (q->initData.sampleRate * q->ticksSmoothed / ERP_TICKS_PER_DEGREE);          // * sample rate / ticks per degree ==> degrees per second
  uint64_t result       = ERP_INCR_SCALE_FACTOR * ERP_TICKS_PER_DEGREE * q->fineThreshold / LIB_InterpolateValue(&(q->table), degPerSecond);
  //printf("%6d ", degPerSecond);
  if (q->initData.adaptiveFiltering)
  {
    // map degPerSecond 0...1000 to lambda/10...lambda
    if (degPerSecond > 290)
      degPerSecond = 290;
    degPerSecond     = 10 + degPerSecond;
    q->workingLambda = q->lambda * degPerSecond / 30.0;
  }
  //printf("%8.7lf %8.7lf\033[1A\n", q->lambda, q->workingLambda);
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

  // determine touch
  // TODO : dependencies on sample rate, buffer size, etc. Allow some user parametrization
#define TOUCH_THRESHOLD (1.5)
#define TOUCH_TIMEOUT   (200)  // 100msec @2kHzSR
  q->bufAverageSmoothed = q->bufAverageSmoothed - (0.00003 * (q->bufAverageSmoothed - (double) q->bufAverage));
  if (q->bufAverageSmoothed > TOUCH_THRESHOLD)
    q->bufAverageSmoothed = TOUCH_THRESHOLD;
  if (q->bufAverageSmoothed < -TOUCH_THRESHOLD)
    q->bufAverageSmoothed = -TOUCH_THRESHOLD;

  if (!q->activity)
  {
    q->activity = fabs(q->bufAverageSmoothed) > TOUCH_THRESHOLD * 0.9;
    if (q->activity)
    {
      if (q->bufAverageSmoothed > 0)
        q->bufAverageSmoothed = TOUCH_THRESHOLD;
      else
        q->bufAverageSmoothed = -TOUCH_THRESHOLD;
    }
  }
  else
  {
    q->activity = fabs(q->bufAverageSmoothed) > TOUCH_THRESHOLD * 0.8;
    if (!q->activity)
      q->bufAverageSmoothed = 0.0;
  }

  if (q->activity)
    q->activityTimer = TOUCH_TIMEOUT;

  printf("%8.2lf \033[1A\n", q->bufAverageSmoothed);

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

  int64_t threshold = lookUpDynamicThreshold(q, ticks);

  q->summedAverage += q->bufAverage;

  int retval = 0;
  if (abs64(q->summedAverage) > threshold)
  {
    retval = (int) (q->summedAverage / threshold);

#if PRECISION_SNAPPOINT
    if (retval >= 0)
      q->summedAverage = q->summedAverage % threshold;
    else
      q->summedAverage = -(-q->summedAverage % threshold);
#else
    q->summedAverage = 0;
#endif
  }
  if (retval != 0)
    q->activityTimer = 1000;

  if (q->activityTimer)
    q->activityTimer--;

  return retval;
}

int ERP_touched(void *const quantizer)
{
  ERP_Quantizer_t *const q = quantizer;
  return q->activityTimer > 0;
}
