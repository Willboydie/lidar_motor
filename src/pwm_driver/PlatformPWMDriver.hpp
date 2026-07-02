#pragma once

#if defined(STM32G4xx)
#include "stm32g4/PWMDriverSTM32G4.hpp"
using PlatformPWMDriver = PWMDriverSTM32G4;
#elif defined(ARDUINO_TEENSY41)
#include "teensy41/PWMDriverTeensy41.hpp"
using PlatformPWMDriver = PWMDriverTeensy41;
#else
#error "PlatformPWMDriver: unsupported target (need STM32G4xx or ARDUINO_TEENSY41)"
#endif
