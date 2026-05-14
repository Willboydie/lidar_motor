// Minimal entry point for the [env:stm32g4] PlatformIO env.
//
// That env exists ONLY to compile-check the STM32G4 PWM driver subtree
// against stm32duino + the HAL/LL APIs. It does NOT build the rest of the
// firmware (interface, sensor, FOC controller, etc.) because those modules
// are still Teensy-specific. When the real STM32G4 port is wired up, this
// stub will be replaced by an actual main.cpp.
#if defined(STM32G4xx)

#include <Arduino.h>
#include "pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"

// Instantiating the driver forces emission of its vtable, which references
// every override (configure3PWM, write3PWM, stopPwmHardware, cleanupParams)
// and so pulls each one through the compiler.
static PWMDriverSTM32G4 _stm32g4_compile_check_driver(0, 1, 2, 3);

void setup() {}
void loop()  {}

#endif
