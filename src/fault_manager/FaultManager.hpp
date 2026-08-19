#pragma once

#include <stdint.h>
#include "../utils/constants.h"
#include "../utils/cmd_defs.h"

static constexpr uint8_t FAULT_CODE_LIMIT = 255;
static constexpr uint8_t FAULT_CODE_MAX = FAULT_CODE_LIMIT;

enum FaultCode : uint8_t {
    FAULT_NONE = 0,
    // --- System / lifecycle (1–15) ---
    FAULT_SYS_INIT_FAILED           = 1,
    FAULT_SYS_NOT_INITIALISED       = 2,
    FAULT_SYS_WATCHDOG              = 3,
    FAULT_SYS_LOOP_OVERRUN          = 4,
    FAULT_SYS_INVALID_MOTOR_NUMBER  = 5,
    // --- Encoder X / AS5048 (16–31) ---
    FAULT_ENC_X_INIT             = 16,
    FAULT_ENC_X_UPDATE            = 17,
    FAULT_ENC_X_SPI_PARITY       = 18,
    FAULT_ENC_X_COF              = 19,
    FAULT_ENC_X_OCF              = 20,
    FAULT_ENC_X_MAG_LOW          = 21,
    FAULT_ENC_X_MAG_HIGH         = 22,
    FAULT_ENC_X_AGC_OUT_OF_RANGE = 23,
    // --- Encoder Y (32–47) ---
    FAULT_ENC_Y_INIT             = 32,
    FAULT_ENC_Y_UPDATE            = 33,
    FAULT_ENC_Y_SPI_PARITY       = 34,
    FAULT_ENC_Y_COF              = 35,
    FAULT_ENC_Y_OCF              = 36,
    FAULT_ENC_Y_MAG_LOW          = 37,
    FAULT_ENC_Y_MAG_HIGH         = 38,
    FAULT_ENC_Y_AGC_OUT_OF_RANGE = 39,
    // --- PWM / driver (48–63) ---
    FAULT_PWM_X_INIT             = 48,
    FAULT_PWM_Y_INIT             = 49,
    FAULT_PWM_SYNC               = 50,
    // --- FOC / motion (64–95) ---
    FAULT_FOC_X_INIT             = 64,
    FAULT_FOC_Y_INIT             = 65,
    FAULT_FOC_X_NAN_COMMAND      = 66,
    FAULT_FOC_Y_NAN_COMMAND      = 67,
    FAULT_FOC_X_TRACKING         = 68,
    FAULT_FOC_Y_TRACKING         = 69,
    FAULT_FOC_X_VEL_LIMIT        = 70,
    FAULT_FOC_Y_VEL_LIMIT        = 71,
    FAULT_MOTION_ESTOP           = 72,
    // --- Current sense (STM32G4) (96–111) ---
    FAULT_CS_X_INIT              = 96,
    FAULT_CS_Y_INIT              = 97,
    FAULT_CS_X_OFFSET            = 98,
    FAULT_CS_Y_OFFSET            = 99,
    FAULT_CS_X_OVERCURRENT       = 100,
    FAULT_CS_Y_OVERCURRENT       = 101,
    // --- LDM / Garmin (112–127) ---
    FAULT_LDM_INIT               = 112,
    FAULT_LDM_I2C                = 113,
    FAULT_LDM_TIMEOUT            = 114,
    FAULT_LDM_INVALID_RANGE      = 115,
    FAULT_LDM_CONFIG             = 116,
    // --- Host / protocol (128–143) ---
    FAULT_CMD_INVALID            = 128,
    FAULT_CMD_REJECTED_STATE     = 129,
    FAULT_PROTO_OVERFLOW         = 130,
    // --- Angle tracker / scan (144–159) ---
    FAULT_TRACKER_X_OVERFLOW     = 144,
    FAULT_TRACKER_Y_OVERFLOW     = 145,
};

class FaultManager {
public:
    FaultManager();
    ~FaultManager();

    bool hasFault() const { return fault_count > 0; }
    uint8_t faulty(FaultCode fault_code) const;

    void addFault(FaultCode fault_code);
    void clearAllFaults();

    void sendErrorMessage(FaultCode fault_code);

private:
    const char* getMessage(FaultCode fault_code) const;

    uint8_t faults[FAULT_CODE_LIMIT] = {};
    uint8_t fault_count = 0;
};

extern FaultManager faultManager;