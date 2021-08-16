#include <stdint.h>
#define INT_MAX (0x7FFFFFFFu)

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ERP.h"

// ----------------------------------------------------

// setup parameters:
// =================

// Larger averaging buffers give more smoothing but also lagging behavior, for a
#define ERP_BUF_SIZE (192)

// This the lowest stable (noise-free) resolution, and yields 10 ticks per degree (or 0.1 degrees/tick)
// Actual dynamic thresholds should range from 0.1x to 10x this value
// 10x (1 tick/degree) gives feasible control of the smallest ticks,
// while 0.1x (100 ticks/degree) allows a 200° turn span a range of 0..20000, for example
#define ERP_THRESHOLD ((ERP_SCALE_FACTOR * ERP_BUF_SIZE) / 3600ll)

static inline int32_t abs32(int32_t const x)
{
  return (x < 0) ? -x : x;
}
static inline int64_t abs64(int64_t const x)
{
  return (x < 0) ? -x : x;
}
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

int ERP_getDynamicIncrement(int const increment)
{

  static int      increments[ERP_BUF_SIZE];
  static unsigned idxI;
  static int64_t  avgdIncrement;
  static int64_t  current;
  static int      fill = 2;
  static int      threshold;
  static int64_t  tickAccu;

  // printf("a=%d\n", newAngle);

  idxI = (idxI + 1) % ERP_BUF_SIZE;
  avgdIncrement -= increments[idxI];
  avgdIncrement += increment;
  increments[idxI] = increment;

  if (fill)
  {
    if (idxI == 0)
      if (!--fill)
      {
        tickAccu = current = avgdIncrement;
        threshold          = ERP_THRESHOLD;
      }
    return 0;
  }

  // determine velocity
  tickAccu += avgdIncrement;
  int ticks = 0;
  if (abs64(tickAccu) > ERP_THRESHOLD)
  {
    ticks = (int) tickAccu / ERP_THRESHOLD;
    if (ticks >= 0)
      tickAccu = tickAccu % ERP_THRESHOLD;
    else
      tickAccu = -(-tickAccu % ERP_THRESHOLD);
  }
  static double ticksSmoothed = 0;
  ticksSmoothed               = ticksSmoothed - (0.02 * (ticksSmoothed - (double) abs(ticks)));  // number of 0.1deg ticks per 500us
  int        degPerSecond     = (int) (2000. * ticksSmoothed / 10.);                             // * sample rate / ticks per degree ==> degrees per second
  static int maxDegPerSecond  = 0;
  if (degPerSecond > maxDegPerSecond)
    maxDegPerSecond = degPerSecond;
  printf("%5d %5d\n\033[1A", degPerSecond, maxDegPerSecond);
  // degPerSecond with typical knob ranges from 0deg/s to about 3000deg/s
  // moderate movements give values ~500deg/s

  // map velocity to resolution (=degrees per increment)
  // - below a certain velocity --> use base resolution
  // - above a certain velocity --> use base resolution times multiplier
  // - in between: interpolate along a curvature (3 curvatures: convex, linear, concave)
  // So, dynamic acceleration has following setup parameters:
  //    min velocity          (>= 0 deg/s)
  //    max velocity          (>= min velocity  && <= 3000deg/s)
  //    base resolution       (>= 0.1deg/incr)
  //    resolution multiplier (>= 1.0)
  //    transition curvature select
  // TODO: use a lookup-table with interpolation
  if (degPerSecond < 5)
    degPerSecond = 5;
  if (degPerSecond > 2000)
    degPerSecond = 2000;

  threshold = (ERP_THRESHOLD * 3000ll) / degPerSecond;

  current += avgdIncrement;
  if (abs64(current) > threshold)
  {
    int retval = (int) (current / threshold);
    //printf("%8d \n\033[1A", retval);
    if (retval >= 0)
      current = current % threshold;
    else
      current = -(-current % threshold);
    return retval;
  }
  return 0;
}
