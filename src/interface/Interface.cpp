#include "Interface.hpp"

Interface::Interface(StateMachine& _stateMachine, FOCController& _focControllerX, FOCController& _focControllerY, Sensor& _sensorX, Sensor& _sensorY, InlineCurrentSense& _currentSenseX, InlineCurrentSense& _currentSenseY, LDM& _ldm, AngleTracker& _angleTrackerX, AngleTracker& _angleTrackerY, ScanPath& _scanPath) 
: stateMachine(_stateMachine)
, focControllerX(_focControllerX)
, focControllerY(_focControllerY)
, sensorX(_sensorX)
, sensorY(_sensorY)
, currentSenseX(_currentSenseX)
, currentSenseY(_currentSenseY)
, ldm(_ldm)
, angleTrackerX(_angleTrackerX)
, angleTrackerY(_angleTrackerY)
, scanPath(_scanPath)
{}


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
        case TARGET_X_SET_CMD:
            handleTargetXSet(data);
            break;
        case TARGET_Y_SET_CMD:
            handleTargetYSet(data);
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
        focControllerX.disable();
        focControllerY.disable();
        ldm.disable();
        stateMachine.enabled = false;
        Serial.println("Scanner disabled");
    } else {
        focControllerX.enable();
        focControllerY.enable();
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
        focControllerX.setControlMode(VELOCITY);
        focControllerY.setControlMode(VELOCITY);
        Serial.println("Control mode toggled to: VELOCITY");
        return;
    }
    else if (stateMachine.controlMode == VELOCITY) {
        stateMachine.controlMode = ANGLE;
        focControllerX.setControlMode(ANGLE);
        focControllerY.setControlMode(ANGLE);
        Serial.println("Control mode toggled to: ANGLE");
        return;
    }
    return;
}


void Interface::handleTargetXSet(float data) {
    if (stateMachine.controlMode == ANGLE) focControllerX.setTargetAngle(data);
    else focControllerX.setTargetVelocity(data);
}

void Interface::handleTargetYSet(float data) {
    if (stateMachine.controlMode == ANGLE) focControllerX.setTargetAngle(data);
    else focControllerY.setTargetVelocity(data);
}

void Interface::handleVelocityPSet(float data) {
    focControllerX.setVelocityP(data);
    focControllerY.setVelocityP(data);
    Serial.println("Velocity P set to: " + String(data));
}

void Interface::handleVelocityISet(float data) {
    focControllerX.setVelocityI(data);
    focControllerY.setVelocityI(data);
    Serial.println("Velocity I set to: " + String(data));
}

void Interface::handleVelocityDSet(float data) {
    focControllerX.setVelocityD(data);
    focControllerY.setVelocityD(data);
    Serial.println("Velocity D set to: " + String(data));
}

void Interface::handleVelocityRampSet(float data) {
    focControllerX.setVelocityRamp(data);
    focControllerY.setVelocityRamp(data);
    Serial.println("Velocity ramp set to: " + String(data));
}

void Interface::handleAnglePSet(float data) {
    focControllerX.setAngleP(data);
    focControllerY.setAngleP(data);
    Serial.println("Angle P set to: " + String(data));
}

void Interface::handleAngleISet(float data) {
    focControllerX.setAngleI(data);
    focControllerY.setAngleI(data);
    Serial.println("Angle I set to: " + String(data));
}

void Interface::handleAngleDSet(float data) {
    focControllerX.setAngleD(data);
    focControllerY.setAngleD(data);
    Serial.println("Angle D set to: " + String(data));
}

void Interface::handleAngleRampSet(float data) {
    focControllerX.setAngleRamp(data);
    focControllerY.setAngleRamp(data);
    Serial.println("Angle ramp set to: " + String(data));
}


void Interface::handleVelocityLPFTauSet(float data) {
    sensorX.setVelocityLPFTau(data);
    sensorY.setVelocityLPFTau(data);
    Serial.println("Velocity LPF tau set to: " + String(data, 8));
}

void Interface::handleAngleLPFTauSet(float data) {
    sensorX.setAngleLPFTau(data);
    sensorY.setAngleLPFTau(data);
    Serial.println("Angle LPF tau set to: " + String(data, 8));
}

void Interface::handleFOCVoltageLPFTauSet(float data) {
    focControllerX.setFOCVoltageLPFTau(data);
    focControllerY.setFOCVoltageLPFTau(data);
    Serial.println("FOC voltage LPF tau set to: " + String(data, 8));
}

void Interface::handleVelocityFeedForwardSet(float data) {
    focControllerX.setVelocityFeedForward(data);
    focControllerY.setVelocityFeedForward(data);
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

void Interface::sendControlPacket() {
    ControlPacket packet = {
        CONTROL_PACKET_HEADER,
        focControllerX.getTargetAngle(),
        focControllerY.getTargetAngle(),
        ldm.getLatestDistance(),
        angleTrackerX.getAssociatedAngle(),
        angleTrackerY.getAssociatedAngle()
    };
    Serial.write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
}   

void Interface::sendStatusPacket() {
    StatusPacket packet = {
        STATUS_PACKET_HEADER,
        static_cast<uint8_t>(stateMachine.enabled),
        static_cast<uint8_t>(stateMachine.controlMode),
        static_cast<uint8_t>(stateMachine.scanning),
        focControllerX.getVelocityP(),
        focControllerX.getVelocityI(),
        focControllerX.getVelocityD(),
        focControllerX.getVelocityRamp(),
        focControllerX.getAngleP(),
        focControllerX.getAngleI(),
        focControllerX.getAngleD(),
        focControllerX.getAngleRamp(),
        sensorX.getVelocityLPFTau(),
        sensorX.getAngleLPFTau(),
        focControllerX.getFOCVoltageLPFTau(),
        focControllerX.getVelocityFeedForward()
        // add health and errors here
    };
    Serial.write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
}

