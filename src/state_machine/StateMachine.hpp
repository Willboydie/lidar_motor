#pragma once

enum ControlMode {
    ANGLE,
    VELOCITY
};


struct StateMachine {
    ControlMode controlMode = ANGLE;
    bool enabled = false;
};
