#ifndef HARDWARE_UTILS_DRIVER_H
#define HARDWARE_UTILS_DRIVER_H

#include "utils/quick_defs.h"
#include <Arduino.h>

#ifndef SIMPLEFOC_PWM_ACTIVE_HIGH
#define SIMPLEFOC_PWM_ACTIVE_HIGH true
#endif
#ifndef SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH
#define SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH true
#endif
#ifndef SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH
#define SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH true
#endif

#define SIMPLEFOC_DRIVER_INIT_FAILED ((void*)-1)

enum PhaseState : uint8_t {
  PHASE_OFF = 0,
  PHASE_ON = 1,
  PHASE_HI = 2,
  PHASE_LO = 3,
};

class SimpleFOCDebug {
public:
  static void enable(Print* debugPrint = &Serial);

  static void println(const __FlashStringHelper* msg);
  static void println(const StringSumHelper msg);
  static void println(const char* msg);
  static void println(const __FlashStringHelper* msg, float val);
  static void println(const char* msg, float val);
  static void println(const __FlashStringHelper* msg, int val);
  static void println(const char* msg, int val);
  static void println(const char* msg, char val);
  static void println();
  static void println(int val);
  static void println(float val);

  static void print(const char* msg);
  static void print(const __FlashStringHelper* msg);
  static void print(const StringSumHelper msg);
  static void print(int val);
  static void print(float val);

protected:
  static Print* _debugPrint;
};

#if defined(SIMPLEFOC_STM32_DEBUG) && !defined(SIMPLEFOC_DISABLE_DEBUG)
#define SIMPLEFOC_DEBUG(msg, ...) SimpleFOCDebug::println(F(msg), ##__VA_ARGS__)
#else
#define SIMPLEFOC_DEBUG(...) ((void)0)
#endif

void* _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC);
void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void* params);

#endif
