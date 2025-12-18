#pragma once

#include <SimpleFOC.h>
#include "../../utils/constants.h"

// angle set point variable
extern float target_angle;
inline void doTarget(char* cmd, Commander& command) { command.scalar(&target_angle, cmd); }
void angle_control_test();