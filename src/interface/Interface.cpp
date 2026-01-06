#include "Interface.hpp"

Interface::Interface(SerialType& _serial, StateMachine& _stateMachine, Motor& _motor1, Motor& _motor2, Sensor& _sensor1, Sensor& _sensor2, LDM& _ldm) 
: serial(_serial)
, stateMachine(_stateMachine)
, motor1(_motor1)
, motor2(_motor2)
, sensor1(_sensor1)
, sensor2(_sensor2)
, ldm(_ldm)
{}

void Interface::init() {
    serial.begin(115200);
    delay(1000);
    serial.println("Starting Control Interface...");
    delay(1000);
}


void Interface::printMotorStatus(Motor& motor) {
    if (!stateMachine.enabled) return;
    serial.print("Mech angle: " + String(motor.getMechanicalAngle(), 3));
    serial.print("  |  E angle: " + String(motor.getElectricalAngle(), 3));
    serial.print("  |  Target A: " + String(motor.getTargetAngle(), 3));
    serial.print("  |  V: " + String(motor.getAngularVelocity(), 3));
    serial.print("  |  A error: " + String(motor.getAngularError(), 3));
    serial.print("  |  Target v: " + String(motor.getTargetVelocity(), 3));
    serial.print("  |  V error: " + String(motor.getVelocityError(), 3));
    serial.print("  |  Target Iq: " + String(motor.getTargetQCurrent(), 3));
    serial.print("  |  Vq: " + String(motor.getVq(), 3));
    serial.println("  |  LDM distance: " + String(ldm.getLatestDistance()));
}


void Interface::printStatus() {
    if (!stateMachine.enabled) return;
    printMotorStatus(motor1);
    printMotorStatus(motor2);
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
        case TOGGLE_CONTROL_MODE_CMD:
            handleToggleControlMode();
            break;
        case TARGET_1_SET_CMD:
            handleTarget1Set(data);
            break;
        case TARGET_2_SET_CMD:
            handleTarget2Set(data);
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
        motor1.disable();
        motor2.disable();
        ldm.disable();
        stateMachine.enabled = false;
        serial.println("Scanner disabled");
    } else {
        motor1.enable();
        motor2.enable();
        ldm.enable();
        stateMachine.enabled = true;
        serial.println("Scanner enabled");
    }
}


void Interface::handleToggleControlMode() {
    stateMachine.controlMode = stateMachine.controlMode == ANGLE ? VELOCITY : ANGLE;
    motor1.setControlMode(stateMachine.controlMode);
    motor2.setControlMode(stateMachine.controlMode);
    serial.println("Control mode toggled to: " + String(stateMachine.controlMode == ANGLE ? "ANGLE" : "VELOCITY"));
}


void Interface::handleTarget1Set(float data) {
    serial.println("Setting Motor 1 target to: " + String(data));
    if (stateMachine.controlMode == ANGLE) motor1.setTargetAngle(data);
    else motor1.setTargetVelocity(data);
}

void Interface::handleTarget2Set(float data) {
    serial.println("Setting Motor 2 target to: " + String(data));
    if (stateMachine.controlMode == ANGLE) motor2.setTargetAngle(data);
    else motor2.setTargetVelocity(data);
}


void Interface::handleVelocityPSet(float data) {
    motor1.setVelocityP(data);
    motor2.setVelocityP(data);
    serial.println("Velocity P set to: " + String(data));
}

void Interface::handleVelocityISet(float data) {
    motor1.setVelocityI(data);
    motor2.setVelocityI(data);
    serial.println("Velocity I set to: " + String(data));
}

void Interface::handleVelocityDSet(float data) {
    motor1.setVelocityD(data);
    motor2.setVelocityD(data);
    serial.println("Velocity D set to: " + String(data));
}

void Interface::handleVelocityRampSet(float data) {
    motor1.setVelocityRamp(data);
    motor2.setVelocityRamp(data);
    serial.println("Velocity ramp set to: " + String(data));
}

void Interface::handleAnglePSet(float data) {
    motor1.setAngleP(data);
    motor2.setAngleP(data);
    serial.println("Angle P set to: " + String(data));
}

void Interface::handleAngleISet(float data) {
    motor1.setAngleI(data);
    motor2.setAngleI(data);
    serial.println("Angle I set to: " + String(data));
}

void Interface::handleAngleDSet(float data) {
    motor1.setAngleD(data);
    motor2.setAngleD(data);
    serial.println("Angle D set to: " + String(data));
}

void Interface::handleAngleRampSet(float data) {
    motor1.setAngleRamp(data);
    motor2.setAngleRamp(data);
    serial.println("Angle ramp set to: " + String(data));
}

void Interface::handleSensorVelocityLPFSet(float data) {
    sensor1.setSensorVelocityLPF(data);
    sensor2.setSensorVelocityLPF(data);
    serial.println("Sensor velocity LPF set to: " + String(data));
}

void Interface::handleFOCVoltageLPFSet(float data) {
    motor1.setFOCVoltageLPF(data);
    motor2.setFOCVoltageLPF(data);
    serial.println("FOC voltage LPF set to: " + String(data));
}