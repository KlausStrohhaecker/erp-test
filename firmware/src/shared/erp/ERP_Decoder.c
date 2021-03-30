#include <stdint.h>
#define INT_MAX (0x7FFFFFFFu)

#include "ERP.h"

/*
 *   Momentary Absolute Angle Retrieval
 *   ---------------------------------
 *
 *               S
 *    100 +-----.+.----.+.-----+------+    +  Wiper Result Weighting:
 *        |    /   \  /   \           |    |    -- ignore
 *     2=>----/-----\C-----D          |    \    -- gradual weight
 *  W     |  /      /\     |\         |     \   /
 *  i     | /W1    /  \    | \        |      |  \
 *  p     |/      /    \   |  \       |      |   |
 *  e  50 +      +      +  |   +      +      |    + full weight
 *  r     |     /        \ |    \    /|      |   |
 *        |    /          \|     \  / |      |  /
 * (%) 1=>----/------------A------\B  |     /   \
 *        |  /W2           |\     /\  |    /    -- gradual weight
 *        | /              | \   /  \ |    |    -- ignore
 *      0 +°-----+------+--|--°+°----°+    +
 *        0     90     180 X  270    360
 *                Angle (degree)
 *
 * The ERP has two Wipers (W1, W2) which output 0%..100% readings, following the trapezoid outline
 * vs. rotation, offset from each other by 90 degrees.
 * Each wiper's value equates to >>two candidate angles<<, at the intersections of the value line with
 * the trapezoid angle mapping. Wiper 1 produces angles from points A and B, wiper 2 produces angles
 * from points C and D.
 *
 * The Wiper 1 <--> Wiper 2 mapping is known, this will be used for a preceeding sanity check
 * if the values (including noise and tolerances) are "correct enough" to produce meaningful results.
 *
 * >>The actual angle is then obtained by comparing the angles and selecting the two which match best<<
 *
 * In this example, these are the angles at A and at D and the resulting angle X is a (weighted) average.
 *
 * The trapezoid shape with its capped tops/bottoms results in angle readings having an highly sensitive
 * uncertainity zone there. Various tolerances will also extend the uncertainity zone. To avoid getting
 * false selections from the proximity value, situations must be avoided where the wrong angle candidate
 * could be chosen because it happened to be closer to the other wiper's candidate angle than the correct one.
 *
 * This is afforded by introducing a relative weighting to the angles obtained from a wiper, depending
 * on the wiper's proximity to the dangerous area (from 0...5% and 95%..100%, approximately) :
 * - Inside that area the angles of a wiper will be discarded for averaging but not for selection.
 * - In the 5%..15% and 85%...95% (approx.) zones a gradual weighting is used.
 * - In the center section the full weight is used.
 * The weighing factors W1 and W2 are relative weights, so result = (A*W1 + B*W2) / (W1+W2).
 * This weighting (think sort of a crossfade) is also required to have smooth monotonic mapping without
 * major wobbles even with larger nonlinearities and notably with mechanical angle offsets in the component.
 *
 * A 10bit ADC (0...1023) will give about 180deg/1024 = 0.18deg (noiseless) resolution as the 0% to 100%
 * slope covers a 180deg span. Natural noise will help as it dithers the signal and with some smoothing
 * higher resolution can be acheived, at the cost of settling time.
 *
 */

// ----------------------------------------------------

// setup parameters:
// =================

// Required wiper curve matching in percent of ADC range for valid angle calculation.
// Matching is limited by:
//  - wiper linearity
//  - wiper to wiper angle offset
//  - general noise floor
// Emprirical value, set to 2x the limits found on the single test specimen so far.
#define ERP_ERP_WIPER_MISMATCH_PCT (4 * 2)

// Required angle matching tolerance in degrees for valid angle calculation.
// This is an additional safeguard for the above
// Emprirical value, set to 2x the limits found on the single test specimen so far.
#define ERP_ERP_ANGLE_MISMATCH_DEG (5 * 2)

// percent margin of wiper end values that are unsafe/ambiguous
// Value estimated from datasheet curves of the Alpha model 'RV112FF' ERP.
#define ERP_WIPER_DANGER_ZONE_PCT (5)

// percent width of the crossfade zone after the danger zone
// Educated guess/trade-off. A short zone will create larger absolute angle slope errors
// in the transition but will reduce noise as both wipers are used fully
// over most of the span.
#define ERP_WIPER_CROSSFADE_ZONE (10)

static int getAngle(int w1_wiperValue, int w2_wiperValue);
static int wiperToSigned(unsigned const adcValue);

int ERP_DecodeWipersToAngle(unsigned const w1, unsigned const w2)
{
  return getAngle(wiperToSigned(w1), wiperToSigned(w2));
}

int ERP_GetAngleDifference(int const angle, int const previousAngle)
{
  int difference = angle - previousAngle;
  if (difference >= ERP_SCALE_FACTOR / 2)
    return difference - ERP_SCALE_FACTOR;
  if (difference <= -ERP_SCALE_FACTOR / 2)
    return difference + ERP_SCALE_FACTOR;
  return difference;
}

//
// ------------------------------------------------------------------------------------
//

#define ERP_WIPER_MISMATCH (ERP_ERP_WIPER_MISMATCH_PCT * ERP_WIPER_RANGE / 100)
#define ERP_ANGLE_MISMATCH (ERP_ERP_ANGLE_MISMATCH_DEG * ERP_ANGLE_360 / 360)

static int wiperToSigned(unsigned const adcValue)
{
  int wiperValue = adcValue - (ERP_WIPER_MAX + 1);  // move unsigned offset type to centered signed
  if (wiperValue < ERP_WIPER_MIN)
    wiperValue = ERP_WIPER_MIN;  // crop off asymmetric neg. end
  return wiperValue;
}

typedef struct
{
  int16_t candidateAngle[2];
  int16_t weight;
} AngleData_t;

static void       mapWiperToAngleCandidates(int const wiperValue, int const wiperSelect, AngleData_t* const result);
static int        angleDiff(int const a, int const b);
static inline int abs(int const x)
{
  return (x < 0) ? -x : x;
}

// return INT_MAX when calculation of angle fails (illegal wiper values)
static int getAngle(int w1_wiperValue, int w2_wiperValue)
{
  // obtain candidate angles and result weight
  AngleData_t w[2];
  mapWiperToAngleCandidates(w1_wiperValue, 0, &w[0]);
  mapWiperToAngleCandidates(w2_wiperValue, 1, &w[1]);

  int const w0 = w[0].weight;
  int const w1 = w[1].weight;

  // sanity check wiper values
  int const wp1 = abs(w1_wiperValue);
  int const wp2 = abs(w2_wiperValue);
  if (w0 | w1)  // check only when both wiper values are outside the danger zones
    if (abs(wp1 - (int) (ERP_WIPER_MAX - wp2)) > ERP_WIPER_MISMATCH)
      return INT_MAX;  // wiper matching is outside allowed tolerance
  if (w0 == 0 && w1 == 0)
    return INT_MAX;  // both wiper values are inside the danger zones

  // find best matching angle pair
  int min, tmp, j, k;
  min = angleDiff(w[0].candidateAngle[0], w[1].candidateAngle[0]), j = k = 0;
  if ((tmp = angleDiff(w[0].candidateAngle[1], w[1].candidateAngle[0])) < min)
    min = tmp, j = 1;
  if ((tmp = angleDiff(w[0].candidateAngle[0], w[1].candidateAngle[1])) < min)
    min = tmp, k = 1;
  if (angleDiff(w[0].candidateAngle[1], w[1].candidateAngle[1]) < min)
    j = k = 1;
  int const ca0 = w[0].candidateAngle[j];
  int const ca1 = w[1].candidateAngle[k];

  if (w0 | w1)  // check only when both wiper values are outside the danger zones
    if (angleDiff(ca0, ca1) > ERP_ANGLE_MISMATCH)
      return INT_MAX;  // angle difference is too large

  // calculate weighted average angle
  if (w0 == w1)
    return (ca0 + ca1) * ERP_MIX_SCALING / 2;  // shortcut for equal weights
  return ((w0 * ca0) + (w1 * ca1)) * ERP_MIX_SCALING / (w0 + w1);
}

#define WIPER_DANGER_ZONE (ERP_WIPER_DANGER_ZONE_PCT * ERP_WIPER_RANGE / 100)
#define WIPER_BLEND_ZONE  (ERP_WIPER_CROSSFADE_ZONE * ERP_WIPER_RANGE / 100)

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

static void mapWiperToAngleCandidates(
    int const          wiperValue,
    int const          wiperSelect,  // == 0 --> W1, != 0 --> W2
    AngleData_t* const result)
{
  // calculate wheight based on distance to edges
  int const distance = ERP_WIPER_MAX - abs(wiperValue);
  result->weight     = MIN(MAX(0, distance - WIPER_DANGER_ZONE), WIPER_BLEND_ZONE);

  // calculate angles
  int const phi = wiperValue;
  int       ca0 = phi;
  int       ca1;
  if (phi > 0)
    ca1 = +ERP_ANGLE_180 - phi;
  else
    ca1 = -ERP_ANGLE_180 - phi;

  if (wiperSelect)
  {  // offset 2nd wiper by +90 deg
    ca0 += ERP_ANGLE_90;
    ca1 += ERP_ANGLE_90;
    if (ca1 >= ERP_ANGLE_180)
      ca1 -= ERP_ANGLE_360;
  }
  result->candidateAngle[0] = ca0;
  result->candidateAngle[1] = ca1;
}

static int angleDiff(int const a, int const b)
{
  int ret = abs(a - b);
  // since angles are modulo [-180, +180[, an angle difference cannot be larger than 180
  if (ret > ERP_ANGLE_180)
    ret = ERP_ANGLE_360 - ret;
  return ret;
}
