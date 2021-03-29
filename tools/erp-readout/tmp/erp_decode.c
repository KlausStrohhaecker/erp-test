#include <malloc.h>
#include <math.h>
#include <limits.h>
//#include <mem.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "erp_decode.h"

/*
 *
 * Principles of Operation
 *
 *   Absolute Momentary Angle Retrival
 *   =======================
 *
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
 * In this example, these are the angles at A and at D and the resulting angle X is a (wheighted) average.
 *
 * The trapezoid shape with its capped tops/bottoms results in angle readings having an highly sensitive
 * uncertainity zone there. Various tolerances will also extend the uncertainity zone. To avoid getting
 * false selections from the proximity value, situations must be avoided where the wrong angle candidate
 * could be choosen because it happened to be closer to the other wiper's candidate angle than the correct one.
 *
 * This is afforded by introducing a relative wheigthing to the angles obtained from a wiper, depending
 * on the wiper's proximity to the dangerous area (from 0...15% and 85%..100%, approximately) :
 * - Inside that area the angles of a wiper will be discarded for averaging but not for selection
 *     (actually both will be replaced with the corresponding 0, 90, 180 or 270 values), the other wiper
 *     will close to the center section and will give stable results.
 * - In the 15%..35% and 65%...85% (approx.) zones a gradual wheighting is used.
 * - In the center section the full weight is used.
 * The weighing factors W1 and W2 are relative wheights, so result = (A*W1 + B*W2) / (W1+W2).
 * This weigthing (think sort of a crossfade) is also required to have smooth monotonic mapping without
 * major wobbles even with larger nonlinearities and notably with mechanical angle offsets in the component.
 *
 * Since the angle values are generated modulo 360deg, wrapping can occur around 0deg and 360deg
 * and must be handled properly in the smoothing etc. Because the original sample rate will be high,
 * the wrapping can always be correctly detected as the angle value will never jump in large steps > 180deg.
 * (it is impossible the ERP could ever be operated a much higher rate than 1 revolution in 200ms or so).
 * Smoothing therefore best done in the incremental domain.
 *
 *
 *   Resolution and Stability
 *   ------------------------
 *
 * A 10bit ADC (0...1023) will give about 180deg/1024 = 0.18deg (noiseless) resolution as the 0% to 100%
 * slope covers a 180deg span. Aiming at a final reasonable resolution in 1deg slices, this leaves room for
 * about +-3LSB's of tolerable noise (after smoothing).
 *
 * The output of the momentary angles will be heavily smoothed and this increases virtual resolution
 * and noise/glitch margin. The limit of possible smoothing is reached when the output lags the rotation
 * significantly, by more than, say 20ms. With a simple sliding window of 64 averages a virtual resolution
 * of 16 bits can be achieved for angle values.
 * Effective sample rate should be such that a step change ramps up to the new value in less than 20ms,
 * so about 3kHz.
 *
 * The analog front-end could be without any low-pass (RC) filters to allow for additional self-dithering
 * from the inherent noise of the raw 10bit ADC values. Actual sample rate will likely be much higher
 * so even heavier averaging could be used.
 *
 *
 *    Generating the Incremental Output
 *    =================================
 *
 * With (smoothed) momentary absolute angles available, stable incremental 1 degree steps can be generated.
 * The absolute angle values are binned into overlapping segments spanning a range around the center
 * output values. The switching between output values has a hysteresis to avoid unstable values once the
 * decision has been made on the current output value.
 *
 * Assume an actual angle of 7.5deg and the output is 7deg. When the input increases above 7.75deg the
 * output switches to 8deg. When the input decreases again, switching back to 7deg will happen when the input
 * goes below 7.25deg, so we have 0.5deg of noise/drift tolerance.
 *
 * Because of the known absolute angle the incremental output can be made non-drifting, that is, moving
 * the ERP to a new position and back to where it came from will produce equal amounts of increments no matter
 * how the exact movement took place. Only the small hysteresis will be present depending on which direction
 * we were coming from to reach the two points.
 *
 *
 *    Application Interface
 *    =====================
 *
 * Two methods would apply:
 * (1) Whenever an increment is produced in the continuously working background driver, a callback is
 *     issued with the increment and elapsed time since the last callback.
 * (2) The applications actively fetches the increment (which might be 0) since the last call, again the
 *     increment and elapsed time are returned.
 *
 *
 *   Implementation Details
 *   ----------------------
 *
 * Both the ADC input values and the angle values will be centered signed integers:
 *   ADC inputs : -511(!)..0..+511, mapping to 0%..50%..100%.
 *   Angles     : a ]-180, +180] interval from a mapping -(MAX+1)..0..(MAX), full 2's complement
 */

#if 0
static inline float ERP_AngleValue_ToFloat(AngleValue_t const angle)  // uint16 ==> [0.0, 360.0[
{
  return (float) angle * 360.0 / ANGLE_RANGE;
}

static inline AdcValue_t ERP_Float_toWiperValue(float x)  // 0.0..1.0 ==> [0..1023]
{
  if (x < 0.0)
    x = 0.0;
  if (x > 1.0)
    x = 1.0;
  return (AdcValue_t)(x * (float) WIPER_RANGE);
}
#endif

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

// ----------------------------------------------

typedef int16_t AdcValue_t;    // 0..1023 ==> [-50%, +50%]
typedef int16_t AngleValue_t;  // 0..65535 ==> [0deg, 360deg[
#define WIPER_RANGE           (1024ll)
#define WIPER_MAX             (WIPER_RANGE / 2 - 1)
#define WIPER_MIN             (-WIPER_MAX)
#define WIPER_DANGER_ZONE_PCT (10ll)
#define WIPER_BLEND_ZONE_PCT  (30ll)
#define WIPER_DANGER_ZONE     (WIPER_DANGER_ZONE_PCT * WIPER_RANGE / 100ll)
#define WIPER_BLEND_ZONE      (WIPER_BLEND_ZONE_PCT * WIPER_RANGE / 100ll)

#define ANGLE_90_PCT_VAL (105ll)  // extraploated percent value for 90 degree etc
#define ANGLE_90         (WIPER_MAX * ANGLE_90_PCT_VAL / 100ll)
#define ANGLE_180        (2 * ANGLE_90)
#define ANGLE_360        (2 * ANGLE_180)

typedef struct
{
  int16_t candidateAngle[2];
  int16_t wheight;
} AngleData_t;

static void getAngleData(
    int                adcValue,     // 0..1023
    int const          wiperSelect,  // == 0 --> W1, != 0 --> W2
    AngleData_t* const result)
{
  int wiperValue = adcValue - (WIPER_MAX + 1);  // move unsigned offset type to centered signed
  if (wiperValue < WIPER_MIN)
    wiperValue = WIPER_MIN;  // crop off asymmetric neg. end

  // calculate wheight based on distance to edges
  int const distance = WIPER_MAX - abs(wiperValue);
  result->wheight    = MAX(0, distance - WIPER_DANGER_ZONE);
  result->wheight    = MIN(result->wheight, WIPER_BLEND_ZONE);

  // calculate angles
  int const phi = wiperValue;
  int       ca0 = phi;
  int       ca1;
  if (phi >= 0)
    ca1 = +ANGLE_180 - phi;
  else
    ca1 = -ANGLE_180 - phi;

  if (wiperSelect)
  {  // offset 2nd wiper
    ca0 += ANGLE_90;
    ca1 += ANGLE_90;
    if (ca1 >= ANGLE_180)
      ca1 -= ANGLE_360;
  }
  result->candidateAngle[0] = ca0;
  result->candidateAngle[1] = ca1;
}

static int angleDiff(int const a, int const b)
{
  int ret = abs(a - b);
  if (ret > ANGLE_180)
    ret = ANGLE_360 - ret;
  return ret;
}

static int getAngle(
    int w1_adcValue,  // 0..1023
    int w2_adcValue)  // 0..1023
{
  AngleData_t w[2];
  getAngleData(w1_adcValue, 0, &w[0]);
  getAngleData(w2_adcValue, 1, &w[1]);

  int min, tmp;
  int j, k;

  min = angleDiff(w[0].candidateAngle[0], w[1].candidateAngle[0]), j = k = 0;
  if ((tmp = angleDiff(w[0].candidateAngle[1], w[1].candidateAngle[0])) < min)
    min = tmp, j = 1;
  if ((tmp = angleDiff(w[0].candidateAngle[0], w[1].candidateAngle[1])) < min)
    min = tmp, k = 1;
  if (angleDiff(w[0].candidateAngle[1], w[1].candidateAngle[1]) < min)
    j = k = 1;

  int const ca0 = w[0].candidateAngle[j];
  int const ca1 = w[1].candidateAngle[k];
  int const w0  = w[0].wheight;
  int const w1  = w[1].wheight;
  if (w0 || w1)
  {
    if (w0 != w1)
      return ((w0 * ca0) + (w1 * ca1)) * 1024 / (w0 + w1);
    return (ca0 + ca1) * 1024 / 2;
  }
  printf("\nError: wiper values not within bounds\n");
  exit(3);
}

static int getAngleIncrement(int const angle)
{
  static int first = 1;
  static int oldAngle;
  if (first)
  {
    first    = 0;
    oldAngle = angle;
    return 0;
  }
  int result = angle - oldAngle;
  if (result >= 1024 * ANGLE_180)
    result = 1024 * ANGLE_360 - result;
  if (result <= -1024 * ANGLE_180)
    result = -1024 * ANGLE_360 - result;

  oldAngle = angle;
  return result;
}

#define BUF_SIZE  (192)
#define THRESHOLD ((ANGLE_360 * 1024ll * BUF_SIZE) / 36000ll)

int64_t ERP_getIncrement(int const w1, int const w2)
{

  static int      increments[BUF_SIZE];
  static unsigned idxI;
  static int      avgdIncrement;
  static int      current;
  static int      fill = 2;
  static int      threshold;

  int newAngle  = getAngle(w1, w2);
  int increment = getAngleIncrement(newAngle);
  // printf("a=%d\n", newAngle);

  idxI = (idxI + 1) % BUF_SIZE;
  avgdIncrement -= increments[idxI];
  avgdIncrement += increment;
  increments[idxI] = increment;

  if (fill)
  {
    if (idxI == 0)
      if (!--fill)
      {
        current   = avgdIncrement;
        threshold = THRESHOLD;
      }
    return 0;
  }

  int velFactor = MAX(100, INT_MAX / (1 + abs(avgdIncrement) * 6000 / BUF_SIZE));
  velFactor     = MIN(200 * 100, velFactor);
  // printf("%8d \n\033[1A", velFactor);

  threshold = THRESHOLD * velFactor / 100;
  current += avgdIncrement;

  if (abs(current) > threshold)
  {
    int retval = current / threshold;
    current    = 0;
    return retval;
  }
  return 0;
}

double ERP_incrementTo360deg(int increment)
{
  return (360. * (double) increment * (double) THRESHOLD / (double) (ANGLE_360 * 1024ll * BUF_SIZE));
}
void ERP_GetData(int const w1, int const w2, int* const pAngle, int* const pIncrement)
{
  if (pAngle && pIncrement)
  {
    *pAngle     = getAngle(w1, w2);
    *pIncrement = getAngleIncrement(*pAngle);
  }
}

double ERP_AngleTo360deg(double angle)
{
  angle = angle / 1024. * 90.0 / ANGLE_90;
  while (angle >= +180)
    angle -= 360;
  while (angle < -180)
    angle += 360;
  return angle;
}

int randN(int N)
{
  return ((double) rand() / (double) RAND_MAX - 0.5) * 2 * N;
}

void test(void)
{
#if 01
  AngleData_t test;
  for (int w = 0; w < WIPER_RANGE; w++)
  {
    getAngleData(w, 1, &test);
    printf("w=%4.1lf, weight=%3d, phi1=%5.2lf, phi2=%5.2lf\n",
           100.0 * (double) w / WIPER_RANGE, test.wheight,
           (double) test.candidateAngle[0] * 90. / ANGLE_90,
           (double) test.candidateAngle[1] * 90. / ANGLE_90);
  }
#endif

  int w1, w2;
  int a, da;

  for (int i = 0; i < 00; i++)
  {
    w1 = 600 + randN(3);
    w2 = 150 + randN(3);
    w1 = (w1 - 500) * WIPER_MAX / 500 + WIPER_RANGE / 2;
    w2 = (w2 - 500) * WIPER_MAX / 500 + WIPER_RANGE / 2;
    a  = getAngle(w1, w2);
    da = getAngleIncrement(a);
    printf("a=%6.3lf, da=%6.3lf\n",
           (double) a / 1024. * 90.0 / ANGLE_90,
           (double) da / 1024. * 90.0 / ANGLE_90);
  }
}
