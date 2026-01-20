#pragma once

#include <Arduino.h>
#include "../utils/constants.h"
#include "../utils/quick_defs.h"
#include "../utils/tweakable_params.h"

class PWMDriver {
public:
  PWMDriver(int _n1, int _n2, int _n3, int _en) {
    n1 = _n1;
    n2 = _n2;
    n3 = _n3;
    en = _en;
  }
  virtual ~PWMDriver() { deinit(); }
    
  bool init();
  void deinit();
  void disable();
  void enable();
  void setPwm(float Va, float Vb, float Vc);
  float getVoltageLimit() { return voltage_limit; }

protected:
  int n1;
  int n2;
  int n3;
  int en;
  void* params;

private:
  float duty_cycleA;
  float duty_cycleB;
  float duty_cycleC;

  float pwm_frequency = PWM_FREQUENCY;
  float voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  float voltage_limit = MOTOR_VOLTAGE_LIMIT;

  virtual void* configureCenterAligned3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) = 0;
  virtual void writeCenterAligned3PWM(float duty_cycleA, float duty_cycleB, float duty_cycleC, void* params) = 0;
  virtual void stopPwmHardware(void* params) = 0;

  bool enabled = false;
};