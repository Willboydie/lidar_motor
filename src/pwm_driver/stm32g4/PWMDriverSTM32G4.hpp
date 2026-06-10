#pragma once

#include "../PWMDriver.hpp"
#include "stm32_mcu.h"

#if defined(STM32G4xx)

class PWMDriverSTM32G4 : public PWMDriver {
public:
  PWMDriverSTM32G4(int _n1, int _n2, int _n3, int _en) : PWMDriver(_n1, _n2, _n3, _en) {}

  TIM_TypeDef* getMasterTimerInstance() const {
    TIM_HandleTypeDef* master = getMasterTimerHandle();
    return master != NULL ? master->Instance : NULL;
  }

  TIM_HandleTypeDef* getMasterTimerHandle() const {
    if (params == nullptr) return NULL;
    auto* p = static_cast<STM32DriverParams*>(params);
    if (p->master_timer != NULL) return p->master_timer;
    return p->timers_handle[0];
  }

  STM32DriverParams* getDriverParams() const {
    return static_cast<STM32DriverParams*>(params);
  }

private:
  void* configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) override;
  void write3PWM(float duty_cycleA, float duty_cycleB, float duty_cycleC, void* params) override;
  void stopPwmHardware(void* params) override;
  void cleanupParams(void* params) override;
};

#endif
