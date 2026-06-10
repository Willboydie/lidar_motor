#pragma once

#include "Arduino.h"
#include "state_machine/StateMachine.hpp"
#include "foc_controller/FOCController.hpp"
#include "motor/Motor.hpp"
#include "ldm/LDM.hpp"
#include "sensor/Sensor.hpp"
#include "scan_path/ScanPath.hpp"
#include "utils/quick_defs.h"
#include "utils/tweakable_params.h"

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

#define TARGET_1_SET_CMD 0b00010000
#define TARGET_2_SET_CMD 0b00010100

// Outgoing binary FOC status packet header (0xAA, distinct from DataStream's 0x55)
#define FOC_STATUS_HEADER 0b10101010

#pragma pack(push, 1)
struct FOCStatusPacket {
    uint8_t header;           // FOC_STATUS_HEADER
    uint8_t motor;            // motor number (1 or 2)
    float angle;              // mechanical angle [rad]
    float raw_velocity;       // unfiltered angular velocity [rad/s]
    float filtered_velocity;  // filtered angular velocity [rad/s]
    float target_angle;       // [rad]
    float target_velocity;    // [rad/s]
    float angular_error;      // [rad]
    float velocity_error;     // [rad/s]
    float target_iq;          // [A]
    float requested_vq;       // [V]
    float measured_iq;        // [A]
    float raw_adc_a;          // current sense A, raw 12-bit code [LSB]
    float raw_adc_b;          // current sense B, raw 12-bit code [LSB]
    float amps_a;             // phase A current [A]
    float amps_b;             // phase B current [A]
    float volts_a;            // phase A sense voltage at pin [V]
    float volts_b;            // phase B sense voltage at pin [V]
};
#pragma pack(pop)


class Interface {
public:
    Interface(
        StateMachine& _stateMachine, 
        FOCController& _focController1, 
        FOCController& _focController2, 
        Sensor& _sensor1, 
        Sensor& _sensor2, 
        LDM& _ldm, 
        ScanPath& _scanPath
    );
    void processCommand(uint8_t cmd, float data);
    void commandListener();
    void printStatus();
    void printFOCStatus(FOCController& focController, Sensor& sensor);
    void sendFOCStatus(FOCController& focController, Sensor& sensor);
    
private:
    void handleDisableEnable();
    void handlePlayPause();
    void handleToggleControlMode();

    void handleTarget1Set(float data);
    void handleTarget2Set(float data);
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
    FOCController& focController1;
    FOCController& focController2;
    Sensor& sensor1;
    Sensor& sensor2;
    LDM& ldm;
    ScanPath& scanPath;
    uint8_t cmd;
    float data; // temporary storage for command data
};