#pragma once

#include "Arduino.h"
#include "state_machine/StateMachine.hpp"
#include "foc_controller/FOCController.hpp"
#include "motor/Motor.hpp"
#include "ldm/LDM.hpp"
#include "sensor/Sensor.hpp"
#include "current_sense/InlineCurrentSense.hpp"
#include "angle_tracker/AngleTracker.hpp"
#include "utils/constants.h"
#include "utils/quick_defs.h"
#include "utils/tweakable_params.h"
#include "fault_manager/FaultManager.hpp"
#include "system_manager/SystemManager.hpp"
#include "scan_engine/coarse/CoarseScan.hpp"
#include "scan_engine/ScanEngine.hpp"
#include "utils/cmd_defs.h"
#include "parser/Parser.hpp"
#include "io/SerialIO.hpp"

// Packet lengths
constexpr uint16_t CONTROL_PACKET_LEN = 27;
constexpr uint16_t STATUS_PACKET_LEN = 21;

#pragma pack(push, 1)
struct ControlPacket {
    uint8_t header;
    uint16_t len;
    float target_angle_x;
    float target_angle_y;
    uint16_t distance;
    float mechanical_angle_x;   // live rotor angle [rad] — motor-control UI / plotter
    float mechanical_angle_y;
    float associated_angle_x;   // LDM sample-time angle [rad] (AngleTracker)
    float associated_angle_y;
    uint8_t reached_p2p_target;
};

struct StatusPacket {
    uint8_t header;
    uint16_t len;
    uint8_t enabled;
    uint8_t scan_mode;
    uint8_t dynamic_routine;
    uint8_t debug_mode;
    uint8_t has_fault;
    float coarse_centre_x;
    float coarse_centre_y;
    float p2p_target_x;
    float p2p_target_y;
};
#pragma pack(pop)


class Interface {
public:
    Interface(
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
    );

    // Parser callback: [cmd][len_lo][len_hi][payload]
    void onCommand(uint8_t cmd, uint8_t* payload, uint32_t length);

    // Pump Serial RX through the frame parser.
    void commandListener();

    void sendControlPacket();
    void sendStatusPacket();

private:
    bool requirePayloadFloats(uint32_t length, uint32_t count);

    void handleSleep();
    void handleWake();
    void handleRestart();
    void handleEnterBootloader();
    void handleGetErrorMessage();
    void handleToggleDebugMode();
    void handleDisableEnable();
    void handleToggleControlMode();
    void handleTargetXSet(float* data);
    void handleTargetYSet(float* data);
    void handleVelocityPXSet(float* data);
    void handleVelocityIXSet(float* data);
    void handleVelocityDXSet(float* data);
    void handleVelocityPYSet(float* data);
    void handleVelocityIYSet(float* data);
    void handleVelocityDYSet(float* data);
    void handleAnglePXSet(float* data);
    void handleAngleIXSet(float* data);
    void handleAngleDXSet(float* data);
    void handleAnglePYSet(float* data);
    void handleAngleIYSet(float* data);
    void handleAngleDYSet(float* data);
    void handleVelocityLPFTauSet(float* data);
    void handleAngleLPFTauSet(float* data);
    void handleFOCVoltageLPFTauSet(float* data);
    void handleVelocityFeedForwardSet(float* data);
    void handleAccelerationFeedForwardSet(float* data);
    void handleReconfigureLDM(float* data);
    void handleXAmplitudeSet(float* data);
    void handleYAmplitudeSet(float* data);
    void handleXFrequencySet(float* data);
    void handleYFrequencySet(float* data);
    void handleXPhaseSet(float* data);
    void handleYPhaseSet(float* data);
    void handleEnableScanEngine();
    void handleDisableScanEngine();
    void handleBeginCoarseScanning();
    void handleStopCoarseScanning();
    void handleBeginPoint2PointScanning();
    void handleStopPoint2PointScanning();
    void handleSetPoint2PointTarget(float* data);
    void handleUpdateCoarseScanCentre(float* data);

    StateMachine& stateMachine;
    FOCController& focControllerX;
    FOCController& focControllerY;
    Sensor& sensorX;
    Sensor& sensorY;
    InlineCurrentSense& currentSenseX;
    InlineCurrentSense& currentSenseY;
    LDM& ldm;
    AngleTracker& angleTrackerX;
    AngleTracker& angleTrackerY;
    FaultManager& faultManager;
    SystemManager& systemManager;
    CoarseScan& coarseScan;
    ScanEngine& scanEngine;

    static constexpr uint32_t CMD_PAYLOAD_MAX = 16;
    uint8_t cmdPayload[CMD_PAYLOAD_MAX] = {};
    SerialIO serialIO;
    Parser parser;
};
