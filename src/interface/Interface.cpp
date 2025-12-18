#include "Interface.hpp"

Interface::Interface(SerialType& _serial, StateMachine& _stateMachine, Motor& _motor, Sensor& _sensor) 
: serial(_serial)
, stateMachine(_stateMachine)
, motor(_motor)
, sensor(_sensor)
{}

void Interface::init() {
    serial.begin(115200);
    delay(1000);
    serial.println("Starting Control Interface...");
    delay(1000);
}


void Interface::printStatus() {
    if (!stateMachine.enabled) return;
    serial.print("Mech angle: " + String(motor.getMechanicalAngle(), 3));
    serial.print("  |  E angle: " + String(motor.getElectricalAngle()));
    serial.print("  |  Target A: " + String(motor.getTargetAngle()));
    serial.print("  |  V: " + String(motor.getAngularVelocity()));
    serial.print("  |  A error: " + String(motor.getAngularError()));
    serial.print("  |  Target v: " + String(motor.getTargetVelocity()));
    serial.print("  |  V error: " + String(motor.getVelocityError()));
    serial.print("  |  Target Iq: " + String(motor.getTargetQCurrent()));
    serial.print("  |  Vq: " + String(motor.getVq()));
    serial.println();
}


void Interface::commandListener() {
    if (serial.available() >= 5) {
        uint8_t cmdByte = serial.read();
        uint8_t buffer[4];
        size_t readCount = serial.readBytes(buffer, 4);
        if (readCount == 4) {
            float value;
            memcpy(&value, buffer, sizeof(float));
            processCommand(cmdByte, value);
        }
    }
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
        case SENSOR_VELOCITY_LPF_TAU_SET_CMD:
            handleSensorVelocityLPFSet(data);
            break;
        case FOC_VOLTAGE_LPF_TAU_SET_CMD:
            handleFOCVoltageLPFSet(data);
            break;
        default:
            serial.println("Invalid command");
    }
}


void Interface::handleDisableEnable() {
    if (stateMachine.enabled) {
        motor.disable();
        stateMachine.enabled = false;
        serial.println("Scanner disabled");
    } else {
        motor.enable();
        stateMachine.enabled = true;
        serial.println("Scanner enabled");
    }
}


void Interface::handleScanPlayPause() {
    if (stateMachine.enabled) {
        if (stateMachine.scanning) stateMachine.scanning = false;
        else stateMachine.scanning = true;
    } else serial.println("Enable the scanner first.");
}


void Interface::handleToggleControlMode() {
    stateMachine.controlMode = stateMachine.controlMode == ANGLE ? VELOCITY : ANGLE;
    motor.setControlMode(stateMachine.controlMode);
    serial.println("Control mode toggled to: " + String(stateMachine.controlMode == ANGLE ? "ANGLE" : "VELOCITY"));
}


void Interface::handleTargetSet(float data) {
    serial.println("Setting target to: " + String(data));
    if (stateMachine.controlMode == ANGLE) motor.setTargetAngle(data);
    else motor.setTargetVelocity(data);
}



void Interface::handleVelocityPSet(float data) {

    motor.setVelocityP(data);
    serial.println("Velocity P set to: " + String(data));
}

void Interface::handleVelocityISet(float data) {
    motor.setVelocityI(data);
    serial.println("Velocity I set to: " + String(data));
}

void Interface::handleVelocityDSet(float data) {
    motor.setVelocityD(data);
    serial.println("Velocity D set to: " + String(data));
}

void Interface::handleVelocityRampSet(float data) {
    motor.setVelocityRamp(data);
    serial.println("Velocity ramp set to: " + String(data));
}

void Interface::handleAnglePSet(float data) {
    motor.setAngleP(data);
    serial.println("Angle P set to: " + String(data));
}

void Interface::handleAngleISet(float data) {
    motor.setAngleI(data);
    serial.println("Angle I set to: " + String(data));
}

void Interface::handleAngleDSet(float data) {
    motor.setAngleD(data);
    serial.println("Angle D set to: " + String(data));
}

void Interface::handleAngleRampSet(float data) {
    motor.setAngleRamp(data);
    serial.println("Angle ramp set to: " + String(data));
}

void Interface::handleSensorVelocityLPFSet(float data) {
    sensor.setSensorVelocityLPF(data);
    serial.println("Sensor velocity LPF set to: " + String(data));
}

void Interface::handleFOCVoltageLPFSet(float data) {
    motor.setFOCVoltageLPF(data);
    serial.println("FOC voltage LPF set to: " + String(data));
}