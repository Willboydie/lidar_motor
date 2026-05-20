#pragma once

#include "../PWMDriver.hpp"

#if defined(STM32G4xx)

class PWMDriverSTM32G4 : public PWMDriver {
public:
  PWMDriverSTM32G4(int _n1, int _n2, int _n3, int _en) : PWMDriver(_n1, _n2, _n3, _en) {}

private:
  void* configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) override;
  void write3PWM(float duty_cycleA, float duty_cycleB, float duty_cycleC, void* params) override;
  void stopPwmHardware(void* params) override;
  void cleanupParams(void* params) override;
};

#endif
