#include <stdint.h>
#define INT_MAX (0x7FFFFFFFu)

#include <stdio.h>

#include "ERP.h"

// ----------------------------------------------------

// setup parameters:
// =================

#define ERP_BUF_SIZE  (192)
#define ERP_THRESHOLD ((ERP_SCALE_FACTOR * ERP_BUF_SIZE) / 36000ll)

static inline int abs(int const x)
{
  return (x < 0) ? -x : x;
}
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

int ERP_getDynamicIncrement(int const increment)
{

  static int      increments[ERP_BUF_SIZE];
  static unsigned idxI;
  static int      avgdIncrement;
  static int      current;
  static int      fill = 2;
  static int      threshold;

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
        current   = avgdIncrement;
        threshold = ERP_THRESHOLD;
      }
    return 0;
  }

  int velFactor = MAX(100, INT_MAX / (1 + abs(avgdIncrement) * 200000 / ERP_BUF_SIZE));
  velFactor     = MIN(200 * 100, velFactor);
  //printf("%8d \n\033[1A", velFactor);
  //printf("%8d \n\033[1A", abs(avgdIncrement) > 80);

  threshold = ERP_THRESHOLD * velFactor / 100;
  current += avgdIncrement;

  if (abs(current) > threshold)
  {
    int retval = current / threshold;
    current    = 0;
    return retval;
  }
  return 0;
}
