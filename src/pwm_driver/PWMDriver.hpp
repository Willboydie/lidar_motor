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

  bool isInitialised() { return initialised; }
  bool isEnabled() { return enabled; }

protected:
  int n1;
  int n2;
  int n3;
  int en;
  void* params = nullptr;

private:
  float duty_cycleA;
  float duty_cycleB;
  float duty_cycleC;

  float pwm_frequency = MOTOR_PWM_FREQUENCY;
  float voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  float voltage_limit = MOTOR_VOLTAGE_LIMIT;

  virtual void* configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) = 0;
  virtual void write3PWM(float duty_cycleA, float duty_cycleB, float duty_cycleC, void* params) = 0;
  virtual void stopPwmHardware(void* params) = 0;
  virtual void cleanupParams(void* params) = 0;

  bool enabled = false;
  bool initialised = false;
};