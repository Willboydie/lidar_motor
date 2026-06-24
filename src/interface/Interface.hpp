#pragma once

#include "Arduino.h"
#include "state_machine/StateMachine.hpp"
#include "foc_controller/FOCController.hpp"
#include "motor/Motor.hpp"
#include "ldm/LDM.hpp"
#include "sensor/Sensor.hpp"
#include "scan_path/ScanPath.hpp"
#include "current_sense/InlineCurrentSense.hpp"
#include "utils/constants.h"
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

#define ELECTRICAL_OFFSET_1_SET_CMD 0b00011000
#define ELECTRICAL_OFFSET_2_SET_CMD 0b00011001

// Outgoing binary status packet header (0xAA, distinct from DataStream's 0x55)
#define FOC_STATUS_HEADER 0b10101010

// One combined packet carries both motors at once. All MotorStatus fields are
// floats normalised to [-1, 1]: currents by MOTOR_CURRENT_LIMIT, phase voltages
// by VOLTAGE_POWER_SUPPLY, raw/cmd/elec angles by _2PI (so 0..2pi maps to 0..1),
// angular error by _PI (so [-pi, pi] maps to [-1, 1]).
#pragma pack(push, 1)
struct MotorStatus {
    float ia, ib, ic;   // inline phase currents (ABC)
    float id, iq;        // transformed currents (DQ)
    float raw_angle;         // raw sensor angle (mechanical)
    float elec_angle;        // measured electrical angle
    float cmd_elec_angle;    // electrical angle implied by target mechanical angle
    float angular_error;     // target - mechanical, shortest path [-pi, pi]
    float va, vb, vc;    // phase voltages as sent to the PWM
};
struct DualMotorStatusPacket {
    uint8_t header;      // FOC_STATUS_HEADER
    MotorStatus m1;      // motor 1 (X)
    MotorStatus m2;      // motor 2 (Y)
};
#pragma pack(pop)

// Normalise raw physical quantities into a [-1, 1] MotorStatus.
inline MotorStatus makeNormalisedMotorStatus(
    float ia, float ib, float ic,
    float id, float iq,
    float raw_angle, float elec_angle, float cmd_elec_angle, float angular_error,
    float va, float vb, float vc
) {
    auto norm = [](float v, float ref) {
        float x = (ref != 0.0f) ? v / ref : 0.0f;
        if (x > 1.0f) return 1.0f;
        if (x < -1.0f) return -1.0f;
        return x;
    };
    MotorStatus s;
    s.ia = norm(ia, MOTOR_CURRENT_LIMIT);
    s.ib = norm(ib, MOTOR_CURRENT_LIMIT);
    s.ic = norm(ic, MOTOR_CURRENT_LIMIT);
    s.id = norm(id, MOTOR_CURRENT_LIMIT);
    s.iq = norm(iq, MOTOR_CURRENT_LIMIT);
    s.raw_angle = norm(raw_angle, _2PI);
    s.elec_angle = norm(elec_angle, _2PI);
    s.cmd_elec_angle = norm(cmd_elec_angle, _2PI);
    s.angular_error = norm(angular_error, _PI);
    s.va = norm(va, VOLTAGE_POWER_SUPPLY);
    s.vb = norm(vb, VOLTAGE_POWER_SUPPLY);
    s.vc = norm(vc, VOLTAGE_POWER_SUPPLY);
    return s;
}

// Build and write one dual-motor packet over Serial (shared by Interface + tests).
inline void sendDualMotorStatusPacket(const MotorStatus& m1, const MotorStatus& m2) {
    DualMotorStatusPacket packet{ FOC_STATUS_HEADER, m1, m2 };
    Serial.write((uint8_t*)&packet, sizeof(packet));
}


class Interface {
public:
    Interface(
        StateMachine& _stateMachine, 
        FOCController& _focController1, 
        FOCController& _focController2, 
        Sensor& _sensor1, 
        Sensor& _sensor2, 
        InlineCurrentSense& _currentSense1, 
        InlineCurrentSense& _currentSense2, 
        LDM& _ldm, 
        ScanPath& _scanPath
    );
    void processCommand(uint8_t cmd, float data);
    void commandListener();
    void printStatus();
    void printFOCStatus(FOCController& focController, Sensor& sensor);
    void sendDualMotorStatus();
    
private:
    void handleDisableEnable();
    void handlePlayPause();
    void handleToggleControlMode();

    void handleTarget1Set(float data);
    void handleTarget2Set(float data);
    void handleElectricalOffset1Set(float data);
    void handleElectricalOffset2Set(float data);
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
    InlineCurrentSense& currentSense1;
    InlineCurrentSense& currentSense2;
    LDM& ldm;
    ScanPath& scanPath;
    uint8_t cmd;
    float data; // temporary storage for command data
};