#include "Interface.hpp"

Interface::Interface(StateMachine& _stateMachine, Motor& _motor) 
: stateMachine(_stateMachine)
, motor(_motor)
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting Control Interface...");
    delay(1000);
}


void Interface::processCommand(uint8_t cmd, float data) {
    switch (cmd) {
        case DISABLE_ENABLE_CMD:
            handleDisableEnable();
            break;
        case SCAN_PLAY_PAUSE_CMD:
            handleScanPlayPause();
            break;
        case TOGGLE_CONTROL_MODE_CMD:
            handleToggleControlMode();
            break;
        case TARGET_SET_CMD:
            handleTargetSet(data);
            break;
        case VELOCITY_P_SET_CMD:
            handleVelocityPSet(data);
            break;
        case VELOCITY_I_SET_CMD:
            handleVelocityISet(data);
            break;
        case VELOCITY_D_SET_CMD:
            handleVelocityDSet(data);
            break;
        case VELOCITY_RAMP_SET_CMD:
            handleVelocityRampSet(data);
            break;
        case ANGLE_P_SET_CMD:
            handleAnglePSet(data);
            break;
        case ANGLE_I_SET_CMD:
            handleAngleISet(data);
            break;
        case ANGLE_D_SET_CMD:
            handleAngleDSet(data);
            break;
        case ANGLE_RAMP_SET_CMD:
            handleAngleRampSet(data);
            break;
        default:
            Serial.println("Invalid command");
    }
}


void Interface::handleDisableEnable() {
    if (stateMachine.enabled) {
        motor.disable();
        stateMachine.enabled = false;
    } else {
        motor.enable();
        stateMachine.enabled = true;
    }
}


void Interface::handleScanPlayPause() {
    if (stateMachine.enabled) {
        if (stateMachine.scanning) stateMachine.scanning = false;
        else stateMachine.scanning = true;
    } else Serial.println("Enable the scanner first.");
}


void Interface::handleToggleControlMode() {
    if (!stateMachine.enabled) {
        stateMachine.controlMode = stateMachine.controlMode == ANGLE ? VELOCITY : ANGLE;
        motor.setControlMode(stateMachine.controlMode);
    } else {
        Serial.println("Disable the scanner first.");
    }
}


void Interface::handleTargetSet(float data) {
    if (!stateMachine.enabled) {
        if (stateMachine.controlMode == ANGLE) motor.setTargetAngle(data);
        else motor.setTargetVelocity(data);
    } else Serial.println("Disable the scanner first.");
}



void Interface::handleVelocityPSet(float data) {
    if (!stateMachine.enabled) motor.setVelocityP(data);
    else Serial.println("Disable the scanner first.");
}

void Interface::handleVelocityISet(float data) {
    if (!stateMachine.enabled) motor.setVelocityI(data);
    else Serial.println("Disable the scanner first.");
}

void Interface::handleVelocityDSet(float data) {
    if (!stateMachine.enabled) motor.setVelocityD(data);
    else Serial.println("Disable the scanner first.");
}

void Interface::handleVelocityRampSet(float data) {
    if (!stateMachine.enabled) motor.setVelocityRamp(data);
}

void Interface::handleAnglePSet(float data) {
    if (!stateMachine.enabled) motor.setAngleP(data);
    else Serial.println("Disable the scanner first.");
}

void Interface::handleAngleISet(float data) {
    if (!stateMachine.enabled) motor.setAngleI(data);
    else Serial.println("Disable the scanner first.");
}

void Interface::handleAngleDSet(float data) {
    if (!stateMachine.enabled) motor.setAngleD(data);
    else Serial.println("Disable the scanner first.");
}

void Interface::handleAngleRampSet(float data) {
    if (!stateMachine.enabled) motor.setAngleRamp(data);
    else Serial.println("Disable the scanner first.");
}
