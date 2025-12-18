#pragma once

#include "Arduino.h"
#include "state_machine/StateMachine.hpp"
#include "motor/Motor.hpp"

// state machine
#define DISABLE_ENABLE_CMD 0b00000001
#define SCAN_PLAY_PAUSE_CMD 0b00000010

// ajustes
#define TOGGLE_CONTROL_MODE_CMD 0b00000011
#define VELOCITY_P_SET_CMD 0b00001000
#define VELOCITY_I_SET_CMD 0b00001001
#define VELOCITY_D_SET_CMD 0b00001010
#define VELOCITY_RAMP_SET_CMD 0b00001011
#define ANGLE_P_SET_CMD 0b00001100
#define ANGLE_I_SET_CMD 0b00001101
#define ANGLE_D_SET_CMD 0b00001110
#define ANGLE_RAMP_SET_CMD 0b00001111

// target setters
#define TARGET_SET_CMD 0b00010000


class Interface {
public:
    Interface(StateMachine& _stateMachine, Motor& _motor);
    void processCommand(uint8_t cmd, float data);

private:
    void handleDisableEnable();
    void handleScanPlayPause();
    void handleToggleControlMode();
    void handleTargetSet(float data);
    void handleVelocityPSet(float data);
    void handleVelocityISet(float data);
    void handleVelocityDSet(float data);
    void handleVelocityRampSet(float data);
    void handleAnglePSet(float data);
    void handleAngleISet(float data);
    void handleAngleDSet(float data);
    void handleAngleRampSet(float data);

    StateMachine& stateMachine;
    Motor& motor;
    uint8_t cmd;
    float data;
};