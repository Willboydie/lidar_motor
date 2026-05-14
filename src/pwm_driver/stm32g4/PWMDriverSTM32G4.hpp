#pragma once
#include "../PWMDriver.hpp"


#if defined(STM32G4xx)
#include <Arduino.h>  // pulls in HAL/LL via stm32duino


// Each motor owns its own TIM handles in `timers_owned`. Only the first
// `num_timers` slots are populated; multiple phases that land on the same
// peripheral share an entry. `timers_handle[phase]` is a per-phase pointer
// into `timers_owned`, so e.g. a motor whose three phases all sit on TIM1
// has `num_timers == 1` and all three `timers_handle[i]` point to
// `&timers_owned[0]`.
typedef struct STM32DriverParams {
  TIM_HandleTypeDef  timers_owned[3];
  TIM_HandleTypeDef* timers_handle[3] = {NULL};
  uint32_t channels[3];
  uint32_t llchannels[3];
  uint8_t num_timers;
  TIM_HandleTypeDef* master_timer = NULL;
} STM32DriverParams;


// Called by the STM32G4 current-sense module to resume the timers after the
// ADC injected conversion has been kicked off. Free function so the
// current-sense code (which only holds the opaque driver params handle) can
// reach it without depending on this class.
void stm32_resume(STM32DriverParams* params);


#else
#pragma message("STM32G4 code not compiled.")
#endif


class PWMDriverSTM32G4: public PWMDriver {

public:
  PWMDriverSTM32G4(int _n1, int _n2, int _n3, int _en) : PWMDriver(_n1, _n2, _n3, _en) {}

private:
  // -- PWMDriver overrides (portable signatures, declared on every target)
  void* configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) override;
  void  write3PWM    (float duty_cycleA, float duty_cycleB, float duty_cycleC, void* params) override;
  void  stopPwmHardware(void* params) override;
  void  cleanupParams  (void* params) override;

#if defined(STM32G4xx)
  // -- pin / timer search + configuration
  int   findBestTimerCombination(int numPins, int pins[], PinMap* pinTimers[]);
  void  initTimerHandle  (TIM_HandleTypeDef* handle, TIM_TypeDef* peripheral);
  void  initChannelPWM   (TIM_HandleTypeDef* handle, PinMap* pinTimer);
  int   checkTimerFrequency(long pwm_frequency, TIM_HandleTypeDef* timers[], uint8_t num_timers);
  TIM_HandleTypeDef* alignTimers(TIM_HandleTypeDef* timers_in[], uint8_t num_timers_in);
  void  pauseAll  (STM32DriverParams* params);

  // -- low-level timer / channel poke helpers
  void     pauseTimer    (TIM_HandleTypeDef* handle);
  void     resumeTimer   (TIM_HandleTypeDef* handle);
  void     refreshTimer  (TIM_HandleTypeDef* handle);
  void     pauseChannel  (TIM_HandleTypeDef* handle, uint32_t llchannel);
  uint32_t setClockAndARR(TIM_HandleTypeDef* handle, uint32_t PWM_freq);
  void     writePwm      (TIM_HandleTypeDef* timer, uint32_t channel, uint32_t value);
  uint32_t getHALChannel (uint32_t channel);
  uint32_t getLLChannel  (PinMap* timer);
  uint8_t  countTimers   (TIM_HandleTypeDef* timers[], uint8_t num_timers);
  uint8_t  distinctTimers(TIM_HandleTypeDef* timers_in[], uint8_t num_timers, TIM_HandleTypeDef* timers_out[]);
  int      getInternalSourceTrigger(TIM_HandleTypeDef* master, TIM_HandleTypeDef* slave);
  uint32_t getTimerClockFreq(TIM_HandleTypeDef* handle);
#endif
};
