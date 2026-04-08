#include "Interface.hpp"

Interface::Interface(StateMachine& _stateMachine, FOCController& _focController1, FOCController& _focController2, Sensor& _sensor1, Sensor& _sensor2, LDM& _ldm, ScanPath& _scanPath) 
: stateMachine(_stateMachine)
, focController1(_focController1)
, focController2(_focController2)
, sensor1(_sensor1)
, sensor2(_sensor2)
, ldm(_ldm)
, scanPath(_scanPath)
{}


void Interface::printFOCStatus(FOCController& focController, Sensor& sensor) {
    if (!stateMachine.enabled) return;
    float target_angle = -focController.getTargetAngle();

    Serial.print("Mech angle: " + String(focController.getMechanicalAngle(), 3));
    // Serial.print(" | E angle: " + String(focController.getElectricalAngle(), 3));
    // Serial.print(" | Target A: " + String(focController.getTargetAngle(), 3));
    Serial.print(" | A err: " + String(focController.getAngularError(), 3));
    // Serial.print(" | Raw Vel: " + String(sensor.getRawAngularVelocity(), 3));
    // Serial.print(" | Fil Vel: " + String(focController.getAngularVelocity(), 3));
    // Serial.print("  |  Target V: " + String(focController.getTargetVelocity(), 3));
    Serial.print(" | V err: " + String(focController.getVelocityError(), 3));
    Serial.print(" | Target Iq: " + String(focController.getTargetQCurrent(), 3));
    // Serial.print("  |  Vq: " + String(focController.getVq(), 3));
    // Serial.print("  |  LDM distance: " + String(ldm.getLatestDistance()));
    Serial.println();
}

void Interface::commandListener() {
    if (Serial.available() >= 5) {
        uint8_t cmdByte = Serial.read();
        char buffer[4];
        size_t readCount = Serial.readBytes(buffer, 4);
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
            handlePlayPause();
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
        case VELOCITY_LPF_TAU_SET_CMD:
            handleVelocityLPFTauSet(data);
            break;
        case ANGLE_LPF_TAU_SET_CMD:
            handleAngleLPFTauSet(data);
            break;
        case FOC_VOLTAGE_LPF_TAU_SET_CMD:
            handleFOCVoltageLPFTauSet(data);
            break;
        case VELOCITY_FF_SET_CMD:
            handleVelocityFeedForwardSet(data);
            break;
        case RECONFIGURE_LDM_CMD:
            handleReconfigureLDM(data);
            break;
        case THETA_PHASE_INC_SET_CMD:
            handleThetaPhaseIncSet(data);
            break;
        case PHI_PHASE_INC_SET_CMD:
            handlePhiPhaseIncSet(data);
            break;
        case THETA_AMPLITUDE_SET_CMD:
            handleThetaAmplitudeSet(data);
            break;
        case PHI_AMPLITUDE_SET_CMD:
            handlePhiAmplitudeSet(data);
            break;
        case THETA_CENTRE_SET_CMD:
            handleThetaCentreSet(data);
            break;
        case PHI_CENTRE_SET_CMD:
            handlePhiCentreSet(data);
            break;
        default:
            Serial.println("Invalid command");
    }
}


void Interface::handleDisableEnable() {
    if (stateMachine.enabled) {
        focController1.disable();
        focController2.disable();
        ldm.disable();
        stateMachine.enabled = false;
        Serial.println("Scanner disabled");
    } else {
        focController1.enable();
        focController2.enable();
        ldm.enable();
        stateMachine.enabled = true;
        Serial.println("Scanner enabled");
    }
}

void Interface::handlePlayPause() {
    if (stateMachine.controlMode != ANGLE) {
        Serial.println("Control mode must be ANGLE to scan");
        return;
    }
    if (stateMachine.scanning) {
        stateMachine.scanning = false;
        Serial.println("Scanning paused");
    } else {
        stateMachine.scanning = true;
        Serial.println("Scanning started");
    }
}


void Interface::handleToggleControlMode() {
    if (stateMachine.controlMode == ANGLE) {
        stateMachine.controlMode = VELOCITY;
        focController1.setControlMode(VELOCITY);
        focController2.setControlMode(VELOCITY);
        Serial.println("Control mode toggled to: VELOCITY");
        return;
    }
    else if (stateMachine.controlMode == VELOCITY) {
        stateMachine.controlMode = ANGLE;
        focController1.setControlMode(ANGLE);
        focController2.setControlMode(ANGLE);
        Serial.println("Control mode toggled to: ANGLE");
        return;
    }
    return;
}


void Interface::handleTarget1Set(float data) {
    Serial.println("Setting Motor 1 target to: " + String(data));
    if (stateMachine.controlMode == ANGLE) focController1.setTargetAngle(data);
    else focController1.setTargetVelocity(data);
}

void Interface::handleTarget2Set(float data) {
    Serial.println("Setting Motor 2 target to: " + String(data));
    if (stateMachine.controlMode == ANGLE) focController2.setTargetAngle(data);
    else focController2.setTargetVelocity(data);
}


void Interface::handleVelocityPSet(float data) {
    focController1.setVelocityP(data);
    focController2.setVelocityP(data);
    Serial.println("Velocity P set to: " + String(data));
}

void Interface::handleVelocityISet(float data) {
    focController1.setVelocityI(data);
    focController2.setVelocityI(data);
    Serial.println("Velocity I set to: " + String(data));
}

void Interface::handleVelocityDSet(float data) {
    focController1.setVelocityD(data);
    focController2.setVelocityD(data);
    Serial.println("Velocity D set to: " + String(data));
}

void Interface::handleVelocityRampSet(float data) {
    focController1.setVelocityRamp(data);
    focController2.setVelocityRamp(data);
    Serial.println("Velocity ramp set to: " + String(data));
}

void Interface::handleAnglePSet(float data) {
    focController1.setAngleP(data);
    focController2.setAngleP(data);
    Serial.println("Angle P set to: " + String(data));
}

void Interface::handleAngleISet(float data) {
    focController1.setAngleI(data);
    focController2.setAngleI(data);
    Serial.println("Angle I set to: " + String(data));
}

void Interface::handleAngleDSet(float data) {
    focController1.setAngleD(data);
    focController2.setAngleD(data);
    Serial.println("Angle D set to: " + String(data));
}

void Interface::handleAngleRampSet(float data) {
    focController1.setAngleRamp(data);
    focController2.setAngleRamp(data);
    Serial.println("Angle ramp set to: " + String(data));
}


void Interface::handleVelocityLPFTauSet(float data) {
    sensor1.setVelocityLPFTau(data);
    sensor2.setVelocityLPFTau(data);
    Serial.println("Velocity LPF tau set to: " + String(data, 8));
}

void Interface::handleAngleLPFTauSet(float data) {
    sensor1.setAngleLPFTau(data);
    sensor2.setAngleLPFTau(data);
    Serial.println("Angle LPF tau set to: " + String(data, 8));
}

void Interface::handleFOCVoltageLPFTauSet(float data) {
    focController1.setFOCVoltageLPFTau(data);
    focController2.setFOCVoltageLPFTau(data);
    Serial.println("FOC voltage LPF tau set to: " + String(data, 8));
}

void Interface::handleVelocityFeedForwardSet(float data) {
    focController1.setVelocityFeedForward(data);
    focController2.setVelocityFeedForward(data);
    Serial.println("Velocity feed-forward set to: " + String(data));
}

void Interface::handleReconfigureLDM(float data) {
    if (stateMachine.enabled) {
        Serial.println("To reconfigure the LDM first disable the system.");
        return;
    }
    int config_num = static_cast<uint16_t>(data);
    Serial.println("Reconfiguring LDM to preset " + String(config_num));
    ldm.reconfigure(config_num);
}

void Interface::handleThetaPhaseIncSet(float data) {
    uint32_t value = static_cast<uint32_t>(data);
    scanPath.setThetaPhaseInc(value);
    Serial.println("Theta phase increment set to: " + String(value));
}

void Interface::handlePhiPhaseIncSet(float data) {
    uint32_t value = static_cast<uint32_t>(data);
    scanPath.setPhiPhaseInc(value);
    Serial.println("Phi phase increment set to: " + String(value));
}

void Interface::handleThetaAmplitudeSet(float data) {
    scanPath.setThetaAmplitude(data);
    Serial.println("Theta amplitude set to: " + String(data));
}

void Interface::handlePhiAmplitudeSet(float data) {
    scanPath.setPhiAmplitude(data);
    Serial.println("Phi amplitude set to: " + String(data));
}

void Interface::handleThetaCentreSet(float data) {
    scanPath.setThetaCentre(data);
    Serial.println("Theta centre set to: " + String(data));
}

void Interface::handlePhiCentreSet(float data) {
    scanPath.setPhiCentre(data);
    Serial.println("Phi centre set to: " + String(data));
}

