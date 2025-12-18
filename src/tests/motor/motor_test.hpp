#pragma once

#include <SimpleFOC.h>
#include "../../utils/constants.h"
#include "../../utils/tweakable_params.h"
#include "Arduino.h"

#define MOTOR_TEST_VOLTAGE_LIMIT 5.0 // Test voltage (start low, adjust higher if needed)
#define MOTOR_TEST_TARGET_VELOCITY 5.0 // rad/s (slow speed for initial testing)

void motor_test();