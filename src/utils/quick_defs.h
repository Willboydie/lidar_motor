#ifndef QUICK_DEFS_H
#define QUICK_DEFS_H

#include "Arduino.h"
#include "constants.h"

// functions
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#ifndef _round
#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
#endif
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define NOT_SET -1234
#define _isset(a) ( (a) != (NOT_SET) )
#define _UNUSED(v) (void) (v)
#define _powtwo(x) (1 << (x))

#define _swap(a, b) { auto temp = a; a = b; b = temp; }


// function approximating the sine calculation by using fixed size array
// uses a 65 element lookup table and interpolation
// thanks to @dekutree for his work on optimizing this
static inline float fast_sin(float a){
    // 16bit integer array for sine lookup. interpolation is used for better precision
    // 16 bit precision on sine value, 8 bit fractional value for interpolation, 6bit LUT size
    // resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -PI,PI for 3217 steps)
    static uint16_t sine_array[65] = {0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,32768};
    int32_t t1, t2;
    unsigned int i = (unsigned int)(a * (64*4*256.0f/_2PI));
    int frac = i & 0xff;
    i = (i >> 8) & 0xff;
    if (i < 64) {
      t1 = (int32_t)sine_array[i]; t2 = (int32_t)sine_array[i+1];
    }
    else if(i < 128) {
      t1 = (int32_t)sine_array[128 - i]; t2 = (int32_t)sine_array[127 - i];
    }
    else if(i < 192) {
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
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return fast_sin(a_sin);
  }
  
  
static inline void fast_sincos(float a, float* s, float* c){
    *s = fast_sin(a);
    *c = fast_cos(a);
  }

#endif