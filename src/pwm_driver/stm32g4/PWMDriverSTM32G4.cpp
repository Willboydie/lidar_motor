#include "PWMDriverSTM32G4.hpp"
#include "../../hardware_api.h"

#if defined(STM32G4xx)

void* PWMDriverSTM32G4::configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
  if (!pwm_frequency || !_isset(pwm_frequency)) {
    pwm_frequency = MOTOR_PWM_FREQUENCY;
  } else {
    pwm_frequency = _constrain(pwm_frequency, 0, MOTOR_PWM_FREQUENCY_MAX);
  }

  void* params = _configure3PWM(pwm_frequency, pinA, pinB, pinC);
  if (params == SIMPLEFOC_DRIVER_INIT_FAILED) {
    return nullptr;
  }
  return params;
}

void PWMDriverSTM32G4::write3PWM(float duty_cycleA, float duty_cycleB, float duty_cycleC, void* params) {
  _writeDutyCycle3PWM(duty_cycleA, duty_cycleB, duty_cycleC, params);
}

void PWMDriverSTM32G4::stopPwmHardware(void* params) {
  _writeDutyCycle3PWM(0.0f, 0.0f, 0.0f, params);
}

void PWMDriverSTM32G4::cleanupParams(void* params) {
  (void)params;
}

#endif
