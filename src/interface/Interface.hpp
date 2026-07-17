#pragma once

#include "Arduino.h"
#include "state_machine/StateMachine.hpp"
#include "foc_controller/FOCController.hpp"
#include "motor/Motor.hpp"
#include "ldm/LDM.hpp"
#include "sensor/Sensor.hpp"
#include "scan_path/ScanPath.hpp"
#include "current_sense/InlineCurrentSense.hpp"
#include "angle_tracker/AngleTracker.hpp"
#include "utils/constants.h"
#include "utils/quick_defs.h"
#include "utils/tweakable_params.h"
#include "fault_manager/FaultManager.hpp"
#include "system_manager/SystemManager.hpp"

// state machine
#define DISABLE_ENABLE_CMD 0b00000001
#define SCAN_PLAY_PAUSE_CMD 0b00000010
#define TOGGLE_CONTROL_MODE_CMD 0b00000011

#define VELOCITY_P_SET_CMD 0b00001000
#define VELOCITY_I_SET_CMD 0b00001001
#define VELOCITY_D_SET_CMD 0b00001010
#define VELOCITY_RAMP_SET_CMD 0b00001011
#define ANGLE_P_SET_CMD 0b00001100
#define ANGLE_I_SET_CMD 0b00001101
#define ANGLE_D_SET_CMD 0b00001110
#define ANGLE_RAMP_SET_CMD 0b00001111

#define VELOCITY_LPF_TAU_SET_CMD 0b00010010
#define ANGLE_LPF_TAU_SET_CMD 0b00010011
#define FOC_VOLTAGE_LPF_TAU_SET_CMD 0b00010001

#define VELOCITY_FF_SET_CMD 0b10010010

#define RECONFIGURE_LDM_CMD 0b00100000

#define THETA_PHASE_INC_SET_CMD 0b00100001
#define PHI_PHASE_INC_SET_CMD 0b00100010
#define THETA_AMPLITUDE_SET_CMD 0b00100011
#define PHI_AMPLITUDE_SET_CMD 0b00100100
#define THETA_CENTRE_SET_CMD 0b00100101
#define PHI_CENTRE_SET_CMD 0b00100110

#define TARGET_X_SET_CMD 0b00010000
#define TARGET_Y_SET_CMD 0b00010100

#define SLEEP_CMD 0b00011000
#define WAKE_CMD 0b00011001
#define RESTART_CMD 0b00011010
#define ENTER_BOOTLOADER_CMD 0b00011011

#define GET_ERROR_MSG_CMD 0b00011100

// Outgoing binary packet headers (distinct from each other and from command bytes)
#define CONTROL_PACKET_HEADER 0b01010101  // 0x55 — hot-path closed-loop telemetry
#define STATUS_PACKET_HEADER  0b10101010  // 0xAA — housekeeping / tuning telemetry

#pragma pack(push, 1)
struct ControlPacket {
    uint8_t header;
    float target_angle_x;
    float target_angle_y;
    uint16_t distance;
    float mechanical_angle_x;   // live rotor angle [rad] — motor-control UI / plotter
    float mechanical_angle_y;
    float associated_angle_x;   // LDM sample-time angle [rad] (AngleTracker)
    float associated_angle_y;
};

struct StatusPacket {
    uint8_t header;
    uint8_t enabled;
    uint8_t control_mode;
    uint8_t scanning;
    float velocity_p;
    float velocity_i;
    float velocity_d;
    float velocity_ramp;
    float angle_p;
    float angle_i;
    float angle_d;
    float angle_ramp;
    float velocity_lpf_tau;
    float angle_lpf_tau;
    float foc_voltage_lpf_tau;
    float velocity_feed_forward;
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
        ScanPath& _scanPath,
        FaultManager& _faultManager,
        SystemManager& _systemManager
    );
    void processCommand(uint8_t cmd, float data);
    void commandListener();
    void sendControlPacket();
    void sendStatusPacket();
    
private:
    void handleSleep();
    void handleWake();
    void handleRestart();
    void handleEnterBootloader();
    void handleGetErrorMessage();
    void handleDisableEnable();
    void handlePlayPause();
    void handleToggleControlMode();
    void handleTargetXSet(float data);
    void handleTargetYSet(float data);
    void handleVelocityPSet(float data);
    void handleVelocityISet(float data);
    void handleVelocityDSet(float data);
    void handleVelocityRampSet(float data);
    void handleAnglePSet(float data);
    void handleAngleISet(float data);
    void handleAngleDSet(float data);
    void handleAngleRampSet(float data);
    void handleVelocityLPFTauSet(float data);
    void handleAngleLPFTauSet(float data);
    void handleFOCVoltageLPFTauSet(float data);
    void handleVelocityFeedForwardSet(float data);
    void handleReconfigureLDM(float data);
    void handleThetaPhaseIncSet(float data);
    void handlePhiPhaseIncSet(float data);
    void handleThetaAmplitudeSet(float data);
    void handlePhiAmplitudeSet(float data);
    void handleThetaCentreSet(float data);
    void handlePhiCentreSet(float data);

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
    ScanPath& scanPath;    
    FaultManager& faultManager;
    SystemManager& systemManager;
    
    uint8_t cmd;
    float data; // temporary storage for command data
};