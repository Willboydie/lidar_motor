#include "Interface.hpp"

Interface::Interface(StateMachine& _stateMachine, FOCController& _focControllerX, FOCController& _focControllerY, Sensor& _sensorX, Sensor& _sensorY, InlineCurrentSense& _currentSenseX, InlineCurrentSense& _currentSenseY, LDM& _ldm, AngleTracker& _angleTrackerX, AngleTracker& _angleTrackerY, ScanPath& _scanPath, FaultManager& _faultManager, SystemManager& _systemManager) 
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
, faultManager(_faultManager)
, systemManager(_systemManager)
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
        } else {
            faultManager.addFault(FAULT_PROTO_OVERFLOW);
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
        case SLEEP_CMD:
            handleSleep();
            break;
        case WAKE_CMD:
            handleWake();
            break;
        case RESTART_CMD:
            handleRestart();
            break;
        case ENTER_BOOTLOADER_CMD:
            handleEnterBootloader();
            break;
        case GET_ERROR_MSG_CMD:
            handleGetErrorMessage();
            break;
        default:
            faultManager.addFault(FAULT_CMD_INVALID);
    }
}


void Interface::handleDisableEnable() {
    if (stateMachine.enabled) {
        systemManager.disable();
    } else {
        systemManager.enable();
    }
}


void Interface::handleSleep() {
    systemManager.sleep();
}

void Interface::handleWake() {
    systemManager.wake();
}

void Interface::handleRestart() {
    systemManager.restart();
}

void Interface::handleEnterBootloader() {
    systemManager.enterBootloader();
}

void Interface::handleGetErrorMessage() {
    for (uint8_t i = 1; i < FAULT_CODE_MAX; i++) {
        FaultCode fault_code = static_cast<FaultCode>(i);
        if (faultManager.faulty(fault_code) > 0) {
            faultManager.sendErrorMessage(fault_code);
        }
    }
    faultManager.clearAllFaults();
}

void Interface::handlePlayPause() {
    if (stateMachine.controlMode != ANGLE) {
        faultManager.addFault(FAULT_CMD_REJECTED_STATE);
        return;
    }
    stateMachine.scanning = !stateMachine.scanning;
}


void Interface::handleToggleControlMode() {
    if (stateMachine.controlMode == ANGLE) {
        stateMachine.controlMode = VELOCITY;
        focControllerX.setControlMode(VELOCITY);
        focControllerY.setControlMode(VELOCITY);
        return;
    }
    else if (stateMachine.controlMode == VELOCITY) {
        stateMachine.controlMode = ANGLE;
        focControllerX.setControlMode(ANGLE);
        focControllerY.setControlMode(ANGLE);
        return;
    }
    return;
}


void Interface::handleTargetXSet(float data) {
    if (stateMachine.controlMode == ANGLE) focControllerX.setTargetAngle(data);
    else focControllerX.setTargetVelocity(data);
}

void Interface::handleTargetYSet(float data) {
    if (stateMachine.controlMode == ANGLE) focControllerY.setTargetAngle(data);
    else focControllerY.setTargetVelocity(data);
}

void Interface::handleVelocityPSet(float data) {
    focControllerX.setVelocityP(data);
    focControllerY.setVelocityP(data);
}

void Interface::handleVelocityISet(float data) {
    focControllerX.setVelocityI(data);
    focControllerY.setVelocityI(data);
}

void Interface::handleVelocityDSet(float data) {
    focControllerX.setVelocityD(data);
    focControllerY.setVelocityD(data);
}

void Interface::handleVelocityRampSet(float data) {
    focControllerX.setVelocityRamp(data);
    focControllerY.setVelocityRamp(data);
}

void Interface::handleAnglePSet(float data) {
    focControllerX.setAngleP(data);
    focControllerY.setAngleP(data);
}

void Interface::handleAngleISet(float data) {
    focControllerX.setAngleI(data);
    focControllerY.setAngleI(data);
}

void Interface::handleAngleDSet(float data) {
    focControllerX.setAngleD(data);
    focControllerY.setAngleD(data);
}

void Interface::handleAngleRampSet(float data) {
    focControllerX.setAngleRamp(data);
    focControllerY.setAngleRamp(data);
}


void Interface::handleVelocityLPFTauSet(float data) {
    sensorX.setVelocityLPFTau(data);
    sensorY.setVelocityLPFTau(data);
}

void Interface::handleAngleLPFTauSet(float data) {
    sensorX.setAngleLPFTau(data);
    sensorY.setAngleLPFTau(data);
}

void Interface::handleFOCVoltageLPFTauSet(float data) {
    focControllerX.setFOCVoltageLPFTau(data);
    focControllerY.setFOCVoltageLPFTau(data);
}

void Interface::handleVelocityFeedForwardSet(float data) {
    focControllerX.setVelocityFeedForward(data);
    focControllerY.setVelocityFeedForward(data);
}

void Interface::handleReconfigureLDM(float data) {
    if (stateMachine.enabled) {
        faultManager.addFault(FAULT_CMD_REJECTED_STATE);
        return;
    }
    int config_num = static_cast<uint16_t>(data);
    ldm.reconfigure(config_num);
}

void Interface::handleThetaPhaseIncSet(float data) {
    uint32_t value = static_cast<uint32_t>(data);
    scanPath.setThetaPhaseInc(value);
}

void Interface::handlePhiPhaseIncSet(float data) {
    uint32_t value = static_cast<uint32_t>(data);
    scanPath.setPhiPhaseInc(value);
}

void Interface::handleThetaAmplitudeSet(float data) {
    scanPath.setThetaAmplitude(data);
}

void Interface::handlePhiAmplitudeSet(float data) {
    scanPath.setPhiAmplitude(data);
}

void Interface::handleThetaCentreSet(float data) {
    scanPath.setThetaCentre(data);
}

void Interface::handlePhiCentreSet(float data) {
    scanPath.setPhiCentre(data);
}

void Interface::sendControlPacket() {
    // Live mechanical angles for the motor-control UI (UI/realtime_plotter.py).
    ControlPacket packet = {
        CONTROL_PACKET_HEADER,
        focControllerX.getTargetAngle(),
        focControllerY.getTargetAngle(),
        ldm.getLatestDistance(),
        sensorX.getLatestRotorAngle(),
        sensorY.getLatestRotorAngle(),
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
    };
    Serial.write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
}
