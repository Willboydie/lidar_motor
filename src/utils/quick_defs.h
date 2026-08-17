#ifndef QUICK_DEFS_H
#define QUICK_DEFS_H

#include "Arduino.h"
#include "constants.h"
#include "tweakable_params.h"

// functions
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#ifndef _round
#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
#endif
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define NOT_SET -1234
#define _isset(a) ( (a) != (NOT_SET) )
#define _UNUSED(v) (void) (v)
#define _powtwo(x) (1 << (x))
#define _swap(a, b) { auto temp = a; a = b; b = temp; }

static inline bool isWithinXAngleLimits(float angle) { return (angle >= DEFAULT_X_LOWER_LIMIT && angle <= DEFAULT_X_UPPER_LIMIT); }
static inline bool isWithinYAngleLimits(float angle) { return (angle >= DEFAULT_Y_LOWER_LIMIT && angle <= DEFAULT_Y_UPPER_LIMIT); }

constexpr float ADAPTED_CPR_MULTIPLIER = (_2PI / static_cast<float>(ADAPTED_CPR));
static inline float toRadians(float angle) { return angle * ADAPTED_CPR_MULTIPLIER; }

// normalizing radian angle to [0,2PI]
static inline float normalise_angle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}

static inline float findAngularError(float a, float b) {
 return normalise_angle(a - b + _PI) - _PI;
}

static constexpr float eps = SOFT_DEADBAND_WIDTH;
static constexpr float one_over_eps = 1.0f / SOFT_DEADBAND_WIDTH;
static inline float softDeadband(float v) {
  return fabs(v) > eps ? v : v * (fabs(v) * one_over_eps);
}
static constexpr float hard_eps = HARD_DEADBAND_WIDTH;
static inline float hardDeadband(float v) { return fabs(v) > eps ? v : 0.0f; }

static inline uint16_t discardBits(uint16_t x) { return (x + (1 << (SENSOR_DISCARD_BITS - 1))) >> SENSOR_DISCARD_BITS; }  // with rounding

// fast cube root: bit-hack guess of x^(-1/3) + two division-free Newton steps
// handles negative inputs; worst rel. error ~0.5% (add a 3rd step for ~2e-5)
static inline float _cuberootApprox(float x) {
  if (x == 0.0f) return 0.0f;
  float a = fabsf(x);
  union { float f; uint32_t i; } y = { a };
  y.i = 0x548c2b4b - y.i / 3;                  // initial guess of a^(-1/3)
  float r = y.f;
  r = r * (1.3333333f - 0.33333333f * a * r * r * r);  // Newton on 1/cbrt
  float c = a * r * r;                         // a * a^(-2/3) = a^(1/3)
  return x < 0.0f ? -c : c;
}
#define _cuberoot(a) (_cuberootApprox(a))


// function approximating the sine calculation by using fixed size array
// uses a 65 element lookup table and interpolation
// thanks to @dekutree for his work on optimizing this
static inline float fast_sin(float a){
  a = normalise_angle(a);
  // 16bit integer array for sine lookup. interpolation is used for better precision
  // 16 bit precision on sine value, 8 bit fractional value for interpolation, 6bit LUT size
  // resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -PI,PI for 3217 steps)
  static uint16_t sine_array[65] = {0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,32768};
  int32_t t1, t2;
  unsigned int i = (unsigned int)(a * (64 * 4 * 256.0f / _2PI));
  int frac = i & 0xff;
  i = (i >> 8) & 0xff;
  if (i < 64) {
    t1 = (int32_t)sine_array[i]; t2 = (int32_t)sine_array[i+1];
  }
  else if (i < 128) {
    t1 = (int32_t)sine_array[128 - i]; t2 = (int32_t)sine_array[127 - i];
  }
  else if (i < 192) {
    t1 = -(int32_t)sine_array[-128 + i]; t2 = -(int32_t)sine_array[-127 + i];
  }
  else {
    t1 = -(int32_t)sine_array[256 - i]; t2 = -(int32_t)sine_array[255 - i];
  }
  return (1.0f/32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}
  
  // function approximating cosine calculation by using fixed size array
  // ~55us (float array)
  // ~56us (int array)
  // precision +-0.005
  // it has to receive an angle in between 0 and 2PI
static inline float fast_cos(float a){
    a = normalise_angle(a);
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return fast_sin(a_sin);
  }
  
  
static inline void fast_sincos(float a, float* s, float* c){
    *s = fast_sin(a);
    *c = fast_cos(a);
  }

#endif