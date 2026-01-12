#pragma once

enum ControlMode {
    PATH,
    ANGLE,
    VELOCITY
};


struct StateMachine {
    ControlMode controlMode = ANGLE;
    bool scanning = false;
    bool enabled = false;
};
