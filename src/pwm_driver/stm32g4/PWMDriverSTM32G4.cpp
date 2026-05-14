#include "PWMDriverSTM32G4.hpp"


#if defined(STM32G4xx)

#include "../../utils/constants.h"


#pragma message("")
#pragma message("STM32G4-DRV: compiling PWMDriverSTM32G4")
#pragma message("")


// minimum ARR (i.e. >= 8-bit) before we warn about poor duty resolution
#define STM32G4_MIN_PWM_RESOLUTION 255


// ============================================================================
//  PWMDriver overrides
// ============================================================================

// Configure 3 center-aligned PWM outputs for one motor. Each phase's pin is
// mapped to a (peripheral, channel) via PinMap_TIM. Phases that land on the
// same peripheral share a TIM handle (stored in the params struct's owning
// array), so a single counter drives multiple phases.
void* PWMDriverSTM32G4::configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
  if (pwm_frequency <= 0)
    pwm_frequency = MOTOR_PWM_FREQUENCY;

  int pins[3] = { pinA, pinB, pinC };
  PinMap* pinTimers[3] = { NULL, NULL, NULL };
  if (findBestTimerCombination(3, pins, pinTimers) < 0) {
    Serial.println("STM32G4-DRV: Failed to configure center-aligned 3-PWM!");
    return nullptr;
  }

  STM32DriverParams* params = new STM32DriverParams{};
  uint8_t handle_count = 0;

  for (int phase = 0; phase < 3; phase++) {
    TIM_TypeDef* peripheral = (TIM_TypeDef*)pinTimers[phase]->peripheral;

    // share a handle with an earlier phase if it landed on the same peripheral
    TIM_HandleTypeDef* handle = NULL;
    for (uint8_t i = 0; i < handle_count; i++) {
      if (params->timers_owned[i].Instance == peripheral) {
        handle = &params->timers_owned[i];
        break;
      }
    }

    // otherwise initialise the next slot for this peripheral
    if (handle == NULL) {
      handle = &params->timers_owned[handle_count++];
      initTimerHandle(handle, peripheral);
      uint32_t arr = setClockAndARR(handle, pwm_frequency);
      if (arr < STM32G4_MIN_PWM_RESOLUTION) {
        Serial.print("STM32G4-DRV: WARN low timer resolution (<8bit): ");
        Serial.println((int)arr + 1);
      }
    }

    params->timers_handle[phase] = handle;
    params->channels[phase]      = STM_PIN_CHANNEL(pinTimers[phase]->function);
    params->llchannels[phase]    = getLLChannel(pinTimers[phase]);
    initChannelPWM(handle, pinTimers[phase]);
  }
  params->num_timers = handle_count;

  TIM_HandleTypeDef* timers_for_align[3] = {
    params->timers_handle[0],
    params->timers_handle[1],
    params->timers_handle[2],
  };
  checkTimerFrequency(pwm_frequency, timers_for_align, 3);
  params->master_timer = alignTimers(timers_for_align, 3);

  Serial.println("STM32G4-DRV: Center-aligned 3-PWM configured.");
  return params;
}


void PWMDriverSTM32G4::write3PWM(float dc_a, float dc_b, float dc_c, void* params) {
  STM32DriverParams* p = (STM32DriverParams*)params;
  uint32_t duty1 = dc_a * (p->timers_handle[0]->Instance->ARR + 1);
  uint32_t duty2 = dc_b * (p->timers_handle[1]->Instance->ARR + 1);
  uint32_t duty3 = dc_c * (p->timers_handle[2]->Instance->ARR + 1);
  if (p->num_timers == 1) LL_TIM_DisableUpdateEvent(p->timers_handle[0]->Instance);
  writePwm(p->timers_handle[0], p->channels[0], duty1);
  writePwm(p->timers_handle[1], p->channels[1], duty2);
  writePwm(p->timers_handle[2], p->channels[2], duty3);
  if (p->num_timers == 1) LL_TIM_EnableUpdateEvent(p->timers_handle[0]->Instance);
}


void PWMDriverSTM32G4::stopPwmHardware(void* params) {
  STM32DriverParams* p = (STM32DriverParams*)params;
  // 1. Disable channel outputs for each phase
  for (int i = 0; i < 3; i++) {
    if (p->timers_handle[i] == NULL) break;
    pauseChannel(p->timers_handle[i], p->llchannels[i]);
  }
  // 2. Stop the underlying timer counters (master if available, else each one)
  pauseAll(p);
}


void PWMDriverSTM32G4::cleanupParams(void* params) {
  delete static_cast<STM32DriverParams*>(params);
}


// Free function (declared in the .hpp) used by the current-sense module to
// resume PWM after kicking off ADC injected conversion.
void stm32_resume(STM32DriverParams* params) {
  if (params->master_timer != NULL) {
    LL_TIM_EnableCounter(params->master_timer->Instance);
  } else {
    for (uint8_t i = 0; i < params->num_timers; i++)
      LL_TIM_EnableCounter(params->timers_owned[i].Instance);
  }
}


// ============================================================================
//  Pin / timer search + configuration
// ============================================================================

// For each Arduino pin in pins[], find the first compatible (timer, channel)
// entry in PinMap_TIM and store it in pinTimers[]. Returns 0 on success, -1
// if any pin has no PWM-capable timer entry.
//
// NOTE: This is a minimal naive implementation that always picks the first
// PinMap entry for each pin (i.e. the default alternate function). It does
// not try to minimise the number of distinct timers across pins. For more
// complex pin combinations a smarter search may be needed.
int PWMDriverSTM32G4::findBestTimerCombination(int numPins, int pins[], PinMap* pinTimers[]) {
  for (int i = 0; i < numPins; i++) {
    pinTimers[i] = NULL;
    PinName pn = digitalPinToPinName(pins[i]);
    PinMap* p = (PinMap*)PinMap_TIM;
    while (p->pin != NC) {
      if (p->pin == pn) {
        pinTimers[i] = p;
        break;
      }
      p++;
    }
    if (pinTimers[i] == NULL) {
      Serial.print("STM32G4-DRV: ERR: no PWM-capable timer for pin ");
      Serial.println(pins[i]);
      return -1;
    }
  }
  return 0;
}


// Initialise one TIM_HandleTypeDef in-place (Init fields + base init + pause).
// Mirrors what HAL would do during MX_TIM_Init() for a center-aligned PWM.
void PWMDriverSTM32G4::initTimerHandle(TIM_HandleTypeDef* handle, TIM_TypeDef* peripheral) {
  handle->Instance               = peripheral;
  handle->Channel                = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
  handle->Lock                   = HAL_UNLOCKED;
  handle->State                  = HAL_TIM_STATE_RESET;
  for (int i = 0; i < 7; i++) handle->hdma[i] = NULL;
  handle->Init.Prescaler         = 0;
  handle->Init.Period            = ((1 << 16) - 1);
  handle->Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED2;
  handle->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  handle->Init.RepetitionCounter = 1;
  enableTimerClock(handle);
  HAL_TIM_Base_Init(handle);
  pauseTimer(handle);
}


// Configure one capture/compare channel (one phase output) on an already-
// initialised TIM handle.
void PWMDriverSTM32G4::initChannelPWM(TIM_HandleTypeDef* handle, PinMap* pinTimer) {
  TIM_OC_InitTypeDef channelOC = {};
  channelOC.OCMode       = TIM_OCMODE_PWM1;
  channelOC.Pulse        = 0;
  channelOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  channelOC.OCFastMode   = TIM_OCFAST_DISABLE;
  channelOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  channelOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  channelOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  uint32_t channel = STM_PIN_CHANNEL(pinTimer->function);
  HAL_TIM_PWM_ConfigChannel(handle, &channelOC, getHALChannel(channel));
  pinmap_pinout(pinTimer->pin, PinMap_TIM);
  LL_TIM_CC_EnableChannel(handle->Instance, getLLChannel(pinTimer));
  if (IS_TIM_BREAK_INSTANCE(handle->Instance)) {
    __HAL_TIM_MOE_ENABLE(handle);
  }
}


// Sanity-check that all distinct timers used by a motor agree on the PWM
// frequency. Returns 0 on success, -1 on mismatch.
int PWMDriverSTM32G4::checkTimerFrequency(long pwm_frequency, TIM_HandleTypeDef* timers[], uint8_t num_timers) {
  TIM_HandleTypeDef* timers_distinct[3];
  uint8_t timer_num = distinctTimers(timers, num_timers, timers_distinct);
  float common_pwm_freq = 0.0f;
  for (int i = 0; i < timer_num; i++) {
    uint32_t freq = getTimerClockFreq(timers_distinct[i]);
    uint32_t arr = timers_distinct[i]->Instance->ARR;
    uint32_t prescaler = timers_distinct[i]->Instance->PSC;
    float pwm_freq = (float)freq / (prescaler + 1.0f) / (arr + 1.0f) / 2.0f;
    if (i == 0) common_pwm_freq = pwm_freq;
    else if (pwm_freq != common_pwm_freq) return -1;
  }
  if ((common_pwm_freq - (float)pwm_frequency) > 1.0f) return -1;
  return 0;
}


// Align all timers used by a motor onto a single master timer (if possible)
// so the slave PWM signals are phase-locked to the master. Returns the master
// handle on success, or NULL if no master could be elected (in which case
// each timer is just resumed independently).
TIM_HandleTypeDef* PWMDriverSTM32G4::alignTimers(TIM_HandleTypeDef* timers_in[], uint8_t num_timers_in) {
  TIM_HandleTypeDef* timers[3];
  int numTimers = distinctTimers(timers_in, num_timers_in, timers);

  if (numTimers > 1) {
    int16_t master_index = -1;
    int triggerEvent = -1;
    for (int i = 0; i < numTimers; i++) {
      if (!IS_TIM_MASTER_INSTANCE(timers[i]->Instance)) continue;
      // don't clobber a timer already configured as TRGO update (e.g. ADC trigger)
      if (timers[i]->Instance->CR2 & LL_TIM_TRGO_UPDATE) continue;
      // check that this timer can drive every other timer in the set
      for (int slave_i = 0; slave_i < numTimers; slave_i++) {
        if (i == slave_i) continue;
        triggerEvent = getInternalSourceTrigger(timers[i], timers[slave_i]);
        if (triggerEvent == -1) break;
      }
      if (triggerEvent == -1) continue;
      master_index = i;
      break;
    }

    if (master_index >= 0) {
      LL_TIM_SetSlaveMode(timers[master_index]->Instance, LL_TIM_SLAVEMODE_DISABLED);
      LL_TIM_SetTriggerOutput(timers[master_index]->Instance, LL_TIM_TRGO_ENABLE);

      for (int slave_index = 0; slave_index < numTimers; slave_index++) {
        if (slave_index == master_index) continue;
        uint32_t trigger = getInternalSourceTrigger(timers[master_index], timers[slave_index]);
        LL_TIM_SetTriggerInput(timers[slave_index]->Instance, trigger);
        LL_TIM_SetSlaveMode(timers[slave_index]->Instance, LL_TIM_SLAVEMODE_GATED);
      }
      for (int i = 0; i < numTimers; i++) {
        refreshTimer(timers[i]);
        if (i != master_index) resumeTimer(timers[i]);
      }
      resumeTimer(timers[master_index]);
      return timers[master_index];
    }
  }

  // single timer, or no master could be elected - just resume each timer
  for (int i = 0; i < numTimers; i++)
    resumeTimer(timers[i]);
  return NULL;
}


// Pause every timer this motor owns (the master if one was elected, otherwise
// each distinct timer in turn). Slaves stop because they're gated off the
// master, so pausing the master alone is sufficient when one exists.
void PWMDriverSTM32G4::pauseAll(STM32DriverParams* params) {
  if (params->master_timer != NULL) {
    pauseTimer(params->master_timer);
  } else {
    for (uint8_t i = 0; i < params->num_timers; i++)
      pauseTimer(&params->timers_owned[i]);
  }
}


// ============================================================================
//  Low-level timer / channel poke helpers
// ============================================================================

void PWMDriverSTM32G4::pauseTimer(TIM_HandleTypeDef* handle) {
  LL_TIM_DisableCounter(handle->Instance);
}


void PWMDriverSTM32G4::resumeTimer(TIM_HandleTypeDef* handle) {
  LL_TIM_EnableCounter(handle->Instance);
}


void PWMDriverSTM32G4::refreshTimer(TIM_HandleTypeDef* handle) {
  handle->Instance->EGR = TIM_EVENTSOURCE_UPDATE;
}


void PWMDriverSTM32G4::pauseChannel(TIM_HandleTypeDef* handle, uint32_t llchannel) {
  LL_TIM_CC_DisableChannel(handle->Instance, llchannel);
}


uint32_t PWMDriverSTM32G4::setClockAndARR(TIM_HandleTypeDef* handle, uint32_t PWM_freq) {
  uint32_t arr_value = 0;
  uint32_t cycles = getTimerClockFreq(handle) / PWM_freq / 2;
  uint32_t prescaler = (cycles / 0x10000) + 1;
  LL_TIM_SetPrescaler(handle->Instance, prescaler - 1);
  uint32_t ticks = cycles / prescaler;
  if (ticks > 0) arr_value = ticks - 1;
  __HAL_TIM_SET_AUTORELOAD(handle, arr_value);
  refreshTimer(handle);
  return arr_value;
}


// Write the duty for one hardware timer channel (CCRx register).
void PWMDriverSTM32G4::writePwm(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t value) {
  switch (channel) {
    case 1: timer->Instance->CCR1 = value; break;
    case 2: timer->Instance->CCR2 = value; break;
    case 3: timer->Instance->CCR3 = value; break;
    case 4: timer->Instance->CCR4 = value; break;
#ifdef LL_TIM_CHANNEL_CH5
    case 5: timer->Instance->CCR5 = value; break;
#endif
#ifdef LL_TIM_CHANNEL_CH6
    case 6: timer->Instance->CCR6 = value; break;
#endif
    default: break;
  }
}


uint32_t PWMDriverSTM32G4::getHALChannel(uint32_t channel) {
  switch (channel) {
    case 1: return TIM_CHANNEL_1;
    case 2: return TIM_CHANNEL_2;
    case 3: return TIM_CHANNEL_3;
    case 4: return TIM_CHANNEL_4;
#ifdef TIM_CHANNEL_5
    case 5: return TIM_CHANNEL_5;
#endif
#ifdef TIM_CHANNEL_6
    case 6: return TIM_CHANNEL_6;
#endif
    default: return (uint32_t)-1;
  }
}


uint32_t PWMDriverSTM32G4::getLLChannel(PinMap* timer) {
  if (STM_PIN_INVERTED(timer->function)) {
    switch (STM_PIN_CHANNEL(timer->function)) {
      case 1: return LL_TIM_CHANNEL_CH1N;
      case 2: return LL_TIM_CHANNEL_CH2N;
      case 3: return LL_TIM_CHANNEL_CH3N;
#if defined(LL_TIM_CHANNEL_CH4N)
      case 4: return LL_TIM_CHANNEL_CH4N;
#endif
      default: return (uint32_t)-1;
    }
  } else {
    switch (STM_PIN_CHANNEL(timer->function)) {
      case 1: return LL_TIM_CHANNEL_CH1;
      case 2: return LL_TIM_CHANNEL_CH2;
      case 3: return LL_TIM_CHANNEL_CH3;
      case 4: return LL_TIM_CHANNEL_CH4;
#ifdef LL_TIM_CHANNEL_CH5
      case 5: return LL_TIM_CHANNEL_CH5;
#endif
#ifdef LL_TIM_CHANNEL_CH6
      case 6: return LL_TIM_CHANNEL_CH6;
#endif
      default: return (uint32_t)-1;
    }
  }
}


uint8_t PWMDriverSTM32G4::countTimers(TIM_HandleTypeDef* timers[], uint8_t num_timers) {
  uint8_t count = 1;
  for (int i = 1; i < num_timers; i++) {
    if (timers[i] == NULL) break;
    bool found = false;
    for (int j = 0; j < i; j++) {
      if (timers[i] == timers[j]) { found = true; break; }
    }
    if (!found) count++;
  }
  return count;
}


uint8_t PWMDriverSTM32G4::distinctTimers(TIM_HandleTypeDef* timers_in[], uint8_t num_timers, TIM_HandleTypeDef* timers_out[]) {
  uint8_t count = 0;
  for (int i = 0; i < num_timers; i++) {
    if (timers_in[i] == NULL) continue;
    bool found = false;
    for (int j = 0; j < count; j++) {
      if (timers_in[i] == timers_out[j]) { found = true; break; }
    }
    if (!found) timers_out[count++] = timers_in[i];
  }
  return count;
}


// STM32G4: appropriate internal trigger for the master/slave timer combination.
// Returns -1 if not supported. Supported master timers: TIM1-TIM5, TIM8.
int PWMDriverSTM32G4::getInternalSourceTrigger(TIM_HandleTypeDef* master, TIM_HandleTypeDef* slave) {
  (void)slave;
  TIM_TypeDef* TIM_master = master->Instance;
#if defined(TIM1) && defined(LL_TIM_TS_ITR0)
  if (TIM_master == TIM1) return LL_TIM_TS_ITR0;
#endif
#if defined(TIM2) && defined(LL_TIM_TS_ITR1)
  else if (TIM_master == TIM2) return LL_TIM_TS_ITR1;
#endif
#if defined(TIM3) && defined(LL_TIM_TS_ITR2)
  else if (TIM_master == TIM3) return LL_TIM_TS_ITR2;
#endif
#if defined(TIM4) && defined(LL_TIM_TS_ITR3)
  else if (TIM_master == TIM4) return LL_TIM_TS_ITR3;
#endif
#if defined(TIM5) && defined(LL_TIM_TS_ITR4)
  else if (TIM_master == TIM5) return LL_TIM_TS_ITR4;
#endif
#if defined(TIM8) && defined(LL_TIM_TS_ITR5)
  else if (TIM_master == TIM8) return LL_TIM_TS_ITR5;
#endif
  return -1;
}


// STM32G4 timer clock frequency (in Hz) for a given timer handle.
uint32_t PWMDriverSTM32G4::getTimerClockFreq(TIM_HandleTypeDef* handle) {
  RCC_ClkInitTypeDef clkconfig = {};
  uint32_t pFLatency = 0U;
  uint32_t uwTimclock = 0U, uwAPBxPrescaler = 0U;

  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  switch (getTimerClkSrc(handle->Instance)) {
    case 1:
      uwAPBxPrescaler = clkconfig.APB1CLKDivider;
      uwTimclock = HAL_RCC_GetPCLK1Freq();
      break;
    case 2:
      uwAPBxPrescaler = clkconfig.APB2CLKDivider;
      uwTimclock = HAL_RCC_GetPCLK2Freq();
      break;
    default:
    case 0:
      return 0;
  }
  // On G4, when APBx prescaler != 1, timer clock is doubled.
  switch (uwAPBxPrescaler) {
    default:
    case RCC_HCLK_DIV1:
      break;
    case RCC_HCLK_DIV2:
    case RCC_HCLK_DIV4:
    case RCC_HCLK_DIV8:
    case RCC_HCLK_DIV16:
      uwTimclock *= 2;
      break;
  }
  return uwTimclock;
}


#endif
