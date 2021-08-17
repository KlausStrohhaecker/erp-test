#pragma once

#include <stdint.h>

#define ERP_ADC_RANGE (1024)  // 10 bits

// Extrapolated percent value where the slope hits 90 degree, at point 'S'.
// See ERP_Decoder.c for details.
// Close fit to the actual ERP will give highest absolute precision.
// Value estimated from datasheet curves of the Alpha model 'RV112FF' ERP.
#define ERP_SLOPE_INTERSECT_PCT (103)

// Scale factor for the weighted averaging.
// Non-critical (check for int overflow, though)
#define ERP_MIX_SCALING (32)

// internal use
#define ERP_WIPER_RANGE  ((long long) (ERP_ADC_RANGE))
#define ERP_WIPER_MAX    (ERP_WIPER_RANGE / 2 - 1)
#define ERP_WIPER_MIN    (-ERP_WIPER_MAX)
#define ERP_ANGLE_90     (ERP_WIPER_MAX * ERP_SLOPE_INTERSECT_PCT / 100)
#define ERP_ANGLE_180    (2 * ERP_ANGLE_90)
#define ERP_ANGLE_360    (2 * ERP_ANGLE_180)
#define ERP_SCALE_FACTOR (ERP_ANGLE_360 * ERP_MIX_SCALING)

// ---------------------------- DECODER --------------------

// convert raw ADC wiper values [0...ERP_ADC_RANGE[ to a [-180, +180[ interval
// returns angle value  on success, else
//  INT_MAX : ADC values are unstable, not within accepted error bounds
int ERP_DecodeWipersToAngle(unsigned const w1, unsigned const w2);

// returns multiplier for mapping to 360 degrees
static inline float ERP_AngleMultiplier360(void)
{
  return 360.0f / ERP_SCALE_FACTOR;
}

// returns the value which represents 360 degrees
static inline int ERP_Scalefactor360(void)
{
  return ERP_SCALE_FACTOR;
}

// returns the distance between two absolute angles [-180, +180[
// can never be larger than 180
int ERP_GetAngleDifference(int const angle, int const previousAngle);

// ---------------------------- QUANTIZER --------------------

typedef enum
{
  ERP_TRANS_CONVEX,
  ERP_TRANS_LINEAR,
  ERP_TRANS_CONCAVE
} decelerationCurve_t;

typedef struct
{
  uint16_t sampleRate;                // Hz, [500...5000]
  uint16_t velocityStart;             // start velocity for deceleration (>= 0, <= 3000 deg/sec), moderate movements give values ~500deg/s
  uint16_t velocityStop;              // stop velocity for deceleration (>= start, <= 3000 deg/sec)
  float    incrementsPerDegreeStart;  // start (fine) resolution in increments per degrees units, <= 10 incr/deg
  float    incrementsPerDegreeStop;   // stop (coarse) resolution in increments per degrees units, normally > start;
  float    splitPointVelocity;        // split point ]0...1[  0.5 -> center
  float    splitPointIncrement;       // split point ]0...1[  0.5 -> center
} ERP_QuantizerInit_t;

struct ERP_Quantizer_t;

void *ERP_InitQuantizer(const ERP_QuantizerInit_t initData);
int   ERP_ExitQuantizer(void *const quantizer);
int   ERP_getDynamicIncrement(void *const quantizer, int const increment);
