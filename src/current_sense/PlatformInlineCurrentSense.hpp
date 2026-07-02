#pragma once

#include "teensy41/InlineCurrentSenseTeensy41.hpp"

// Zero-current stub for bench FOC tests on Teensy 4.1 and STM32G4.
// Production STM32 firmware uses InlineCurrentSenseSTM32G4 instead.
using PlatformInlineCurrentSense = InlineCurrentSenseTeensy41;
