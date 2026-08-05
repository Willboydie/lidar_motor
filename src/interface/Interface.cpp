#include "Interface.hpp"
#include <cstring>

Interface::Interface(
    StateMachine& _stateMachine,
    FOCController& _focControllerX,
    FOCController& _focControllerY,
    Sensor& _sensorX,
    Sensor& _sensorY,
    InlineCurrentSense& _currentSenseX,
    InlineCurrentSense& _currentSenseY,
    LDM& _ldm,
    AngleTracker& _angleTrackerX,
    AngleTracker& _angleTrackerY,
    FaultManager& _faultManager,
    SystemManager& _systemManager,
    CoarseScan& _coarseScan,
    ScanEngine& _scanEngine
)
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
, faultManager(_faultManager)
, systemManager(_systemManager)
, coarseScan(_coarseScan)
, scanEngine(_scanEngine)
, serialIO(Serial)
, parser(*this, cmdPayload, CMD_PAYLOAD_MAX)
{}

void Interface::commandListener() {
    parser.pump(serialIO);
}

bool Interface::requirePayloadFloats(uint32_t length, uint32_t count) {
    if (length < count * sizeof(float)) {
        faultManager.addFault(FAULT_PROTO_OVERFLOW);
        return false;
    }
    return true;
}

void Interface::onCommand(uint8_t cmd, uint8_t* payload, uint32_t length) {
    float data[4] = {};
    const uint32_t floatCount = length / sizeof(float);
    const uint32_t nCopy = floatCount > 4 ? 4 : floatCount;
    if (payload != nullptr && nCopy > 0) {
        memcpy(data, payload, nCopy * sizeof(float));
    }

    switch (cmd) {
        case DISABLE_ENABLE_CMD:
            handleDisableEnable();
            break;
        case TOGGLE_CONTROL_MODE_CMD:
            handleToggleControlMode();
            break;
        case TARGET_X_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleTargetXSet(data);
            break;
        case TARGET_Y_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleTargetYSet(data);
            break;
        case VELOCITY_P_X_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleVelocityPXSet(data);
            break;
        case VELOCITY_I_X_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleVelocityIXSet(data);
            break;
        case VELOCITY_D_X_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleVelocityDXSet(data);
            break;
        case VELOCITY_P_Y_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleVelocityPYSet(data);
            break;
        case VELOCITY_I_Y_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleVelocityIYSet(data);
            break;
        case VELOCITY_D_Y_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleVelocityDYSet(data);
            break;
        case ANGLE_P_X_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleAnglePXSet(data);
            break;
        case ANGLE_I_X_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleAngleIXSet(data);
            break;
        case ANGLE_D_X_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleAngleDXSet(data);
            break;
        case ANGLE_P_Y_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleAnglePYSet(data);
            break;
        case ANGLE_I_Y_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleAngleIYSet(data);
            break;
        case ANGLE_D_Y_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleAngleDYSet(data);
            break;
        case VELOCITY_LPF_TAU_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleVelocityLPFTauSet(data);
            break;
        case ANGLE_LPF_TAU_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleAngleLPFTauSet(data);
            break;
        case FOC_VOLTAGE_LPF_TAU_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleFOCVoltageLPFTauSet(data);
            break;
        case VELOCITY_FF_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleVelocityFeedForwardSet(data);
            break;
        case ACCELERATION_FF_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleAccelerationFeedForwardSet(data);
            break;
        case RECONFIGURE_LDM_CMD:
            if (requirePayloadFloats(length, 1)) handleReconfigureLDM(data);
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
        case SET_DEBUG_MODE_CMD:
            handleToggleDebugMode();
            break;
        case X_AMPLITUDE_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleXAmplitudeSet(data);
            break;
        case Y_AMPLITUDE_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleYAmplitudeSet(data);
            break;
        case X_FREQUENCY_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleXFrequencySet(data);
            break;
        case Y_FREQUENCY_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleYFrequencySet(data);
            break;
        case X_PHASE_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleXPhaseSet(data);
            break;
        case Y_PHASE_SET_CMD:
            if (requirePayloadFloats(length, 1)) handleYPhaseSet(data);
            break;
        case ENABLE_SCANENGINE_CMD:
            handleEnableScanEngine();
            break;
        case DISABLE_SCANENGINE_CMD:
            handleDisableScanEngine();
            break;
        case BEGIN_COARSE_SCANNING_CMD:
            handleBeginCoarseScanning();
            break;
        case STOP_COARSE_SCANNING_CMD:
            handleStopCoarseScanning();
            break;
        case BEGIN_POINT2POINT_SCANNING_CMD:
            handleBeginPoint2PointScanning();
            break;
        case STOP_POINT2POINT_SCANNING_CMD:
            handleStopPoint2PointScanning();
            break;
        case SET_POINT2POINT_TARGET_CMD:
            if (requirePayloadFloats(length, 2)) handleSetPoint2PointTarget(data);
            break;
        case UPDATE_COARSE_SCAN_CENTRE_CMD:
            if (requirePayloadFloats(length, 2)) handleUpdateCoarseScanCentre(data);
            break;
        default:
            faultManager.addFault(FAULT_CMD_INVALID);
    }
}

// ------------------- Data Packets --------------------------
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
        static_cast<uint8_t>(scanEngine.getScanMode()),
        static_cast<uint8_t>(scanEngine.getDynamicRoutine()),
        static_cast<uint8_t>(Debug::mode),
        static_cast<uint8_t>(faultManager.hasFault())
        static_cast<float>(scanEngine.getCoarseScanCentreX()),
        static_cast<float>(scanEngine.getCoarseScanCentreY()),
        static_cast<float>(scanEngine.getPoint2PointTargetX()),
        static_cast<float>(scanEngine.getPoint2PointTargetY())
    };
    Serial.write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
}


// ------------------- Command Handlers --------------------------
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

void Interface::handleToggleDebugMode() {
    if (Debug::mode) {
        Debug::msg("Debug mode disabled.");
        Debug::setMode(false);
    } else {
        Debug::setMode(true);
        Debug::msg("Debug mode enabled.");
    }
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


void Interface::handleTargetXSet(float* data) {
    if (stateMachine.controlMode == ANGLE) focControllerX.setTargetAngle(data[0]);
    else focControllerX.setTargetVelocity(data[0]);
}

void Interface::handleTargetYSet(float* data) {
    if (stateMachine.controlMode == ANGLE) focControllerY.setTargetAngle(data[0]);
    else focControllerY.setTargetVelocity(data[0]);
}

void Interface::handleVelocityPXSet(float* data) {
    focControllerX.setVelocityP(data[0]);
}

void Interface::handleVelocityIXSet(float* data) {
    focControllerX.setVelocityI(data[0]);
}

void Interface::handleVelocityDXSet(float* data) {
    focControllerX.setVelocityD(data[0]);
}

void Interface::handleVelocityPYSet(float* data) {
    focControllerY.setVelocityP(data[0]);
}

void Interface::handleVelocityIYSet(float* data) {
    focControllerY.setVelocityI(data[0]);
}

void Interface::handleVelocityDYSet(float* data) {
    focControllerY.setVelocityD(data[0]);
}

void Interface::handleAnglePXSet(float* data) {
    focControllerX.setAngleP(data[0]);
}

void Interface::handleAngleIXSet(float* data) {
    focControllerX.setAngleI(data[0]);
}

void Interface::handleAngleDXSet(float* data) {
    focControllerX.setAngleD(data[0]);
}

void Interface::handleAnglePYSet(float* data) {
    focControllerY.setAngleP(data[0]);
}

void Interface::handleAngleIYSet(float* data) {
    focControllerY.setAngleI(data[0]);
}

void Interface::handleAngleDYSet(float* data) {
    focControllerY.setAngleD(data[0]);
}


void Interface::handleVelocityLPFTauSet(float* data) {
    sensorX.setVelocityLPFTau(data[0]);
    sensorY.setVelocityLPFTau(data[0]);
}

void Interface::handleAngleLPFTauSet(float* data) {
    sensorX.setAngleLPFTau(data[0]);
    sensorY.setAngleLPFTau(data[0]);
}

void Interface::handleFOCVoltageLPFTauSet(float* data) {
    focControllerX.setFOCVoltageLPFTau(data[0]);
    focControllerY.setFOCVoltageLPFTau(data[0]);
}

void Interface::handleVelocityFeedForwardSet(float* data) {
    focControllerX.setVelocityFeedForward(data[0]);
    focControllerY.setVelocityFeedForward(data[0]);
}

void Interface::handleAccelerationFeedForwardSet(float* data) {
    focControllerX.setAccelerationFeedForward(data[0]);
    focControllerY.setAccelerationFeedForward(data[0]);
}

void Interface::handleReconfigureLDM(float* data) {
    if (stateMachine.enabled) {
        faultManager.addFault(FAULT_CMD_REJECTED_STATE);
        return;
    }
    int config_num = static_cast<uint32_t>(data[0]);
    ldm.reconfigure(config_num);
}

void Interface::handleXAmplitudeSet(float* data) {
    coarseScan.setXAmplitude(data[0]);
}

void Interface::handleYAmplitudeSet(float* data) {
    coarseScan.setYAmplitude(data[0]);
}

void Interface::handleXFrequencySet(float* data) {
    coarseScan.setXFreq(data[0]);
}

void Interface::handleYFrequencySet(float* data) {
    coarseScan.setYFreq(data[0]);
}

void Interface::handleXPhaseSet(float* data) {
    coarseScan.setXPhase(data[0]);
}

void Interface::handleYPhaseSet(float* data) {
    coarseScan.setYPhase(data[0]);
}

void Interface::handleEnableScanEngine() {
    scanEngine.on();
}

void Interface::handleDisableScanEngine() {
    scanEngine.off();
}

void Interface::handleBeginCoarseScanning() {
    scanEngine.beginCoarseScanning();
}

void Interface::handleStopCoarseScanning() {
    scanEngine.stopCoarseScanning();
}

void Interface::handleBeginPoint2PointScanning() {
    scanEngine.beginPoint2PointScanning();
}

void Interface::handleStopPoint2PointScanning() {
    scanEngine.stopPoint2PointScanning();
}

void Interface::handleSetPoint2PointTarget(float* data) {
    scanEngine.setNewPoint2PointTarget(data[0], data[1]);
}

void Interface::handleUpdateCoarseScanCentre(float* data) {
    scanEngine.updateCoarseScanCentre(data[0], data[1]);
}