#include "FaultManager.hpp"
#include <Arduino.h>
#include <string.h>

FaultManager faultManager;

FaultManager::FaultManager() = default;
FaultManager::~FaultManager() = default;

uint8_t FaultManager::faulty(FaultCode fault_code) const {
   if (fault_code == FAULT_NONE || static_cast<uint8_t>(fault_code) >= FAULT_CODE_MAX) return 0;

   return faults[fault_code];
}

void FaultManager::addFault(FaultCode fault_code) {
    if (fault_code == FAULT_NONE || static_cast<uint8_t>(fault_code) >= FAULT_CODE_MAX) {
        return;
    }
    if (faults[fault_code] == 0) {
        faults[fault_code] = 1;
        ++fault_count;
    }
}


void FaultManager::clearAllFaults() {
    memset(faults, 0, sizeof(faults));
    fault_count = 0;
}


const char* FaultManager::getMessage(FaultCode fault_code) const {
    switch (static_cast<uint8_t>(fault_code)) {
        case FAULT_NONE:                      return "none \n";
        case FAULT_SYS_INIT_FAILED:           return "system init failed \n";
        case FAULT_SYS_NOT_INITIALISED:       return "system not initialised \n";
        case FAULT_SYS_WATCHDOG:              return "watchdog \n";
        case FAULT_SYS_LOOP_OVERRUN:          return "control loop overrun \n";
        case FAULT_SYS_INVALID_MOTOR_NUMBER:  return "invalid motor number \n";
        case FAULT_ENC_X_INIT:                return "encoder X init failed \n";
        case FAULT_ENC_X_UPDATE:            return "encoder X update failed \n";
        case FAULT_ENC_X_SPI_PARITY:       return "encoder X SPI parity \n";
        case FAULT_ENC_X_COF:              return "encoder X COF \n";
        case FAULT_ENC_X_OCF:              return "encoder X OCF \n";
        case FAULT_ENC_X_MAG_LOW:          return "encoder X mag low \n";
        case FAULT_ENC_X_MAG_HIGH:         return "encoder X mag high \n";
        case FAULT_ENC_X_AGC_OUT_OF_RANGE: return "encoder X AGC out of range \n";
        case FAULT_ENC_Y_INIT:             return "encoder Y init failed \n";
        case FAULT_ENC_Y_UPDATE:            return "encoder Y update failed \n";
        case FAULT_ENC_Y_SPI_PARITY:       return "encoder Y SPI parity \n";
        case FAULT_ENC_Y_COF:              return "encoder Y COF \n";
        case FAULT_ENC_Y_OCF:              return "encoder Y OCF \n";
        case FAULT_ENC_Y_MAG_LOW:          return "encoder Y mag low \n";
        case FAULT_ENC_Y_MAG_HIGH:         return "encoder Y mag high \n";
        case FAULT_ENC_Y_AGC_OUT_OF_RANGE: return "encoder Y AGC out of range \n";
        case FAULT_PWM_X_INIT:             return "PWM X init failed \n";
        case FAULT_PWM_Y_INIT:             return "PWM Y init failed \n";
        case FAULT_PWM_SYNC:               return "PWM sync failed \n";
        case FAULT_FOC_X_INIT:             return "FOC X init failed \n";
        case FAULT_FOC_Y_INIT:             return "FOC Y init failed \n";
        case FAULT_FOC_X_NAN_COMMAND:      return "FOC X non-finite command \n";
        case FAULT_FOC_Y_NAN_COMMAND:      return "FOC Y non-finite command \n";
        case FAULT_FOC_X_TRACKING:         return "FOC X tracking error \n";
        case FAULT_FOC_Y_TRACKING:         return "FOC Y tracking error \n";
        case FAULT_FOC_X_VEL_LIMIT:        return "FOC X velocity limit \n";
        case FAULT_FOC_Y_VEL_LIMIT:        return "FOC Y velocity limit \n";
        case FAULT_MOTION_ESTOP:           return "motion e-stop \n";
        case FAULT_CS_X_INIT:              return "current sense X init failed \n";
        case FAULT_CS_Y_INIT:              return "current sense Y init failed \n";
        case FAULT_CS_X_OFFSET:            return "current sense X offset \n";
        case FAULT_CS_Y_OFFSET:            return "current sense Y offset \n";
        case FAULT_CS_X_OVERCURRENT:       return "current sense X overcurrent \n";
        case FAULT_CS_Y_OVERCURRENT:       return "current sense Y overcurrent \n";
        case FAULT_LDM_INIT:               return "LDM init failed \n";
        case FAULT_LDM_I2C:                return "LDM I2C nack \n";
        case FAULT_LDM_TIMEOUT:            return "LDM timeout \n";
        case FAULT_LDM_INVALID_RANGE:      return "LDM invalid range \n";
        case FAULT_LDM_CONFIG:             return "LDM config rejected \n";
        case FAULT_CMD_INVALID:            return "invalid command \n";
        case FAULT_CMD_REJECTED_STATE:     return "command rejected in current state \n";
        case FAULT_PROTO_OVERFLOW:         return "serial protocol framing error \n";
        case FAULT_TRACKER_X_OVERFLOW:     return "angle tracker X overflow \n";
        case FAULT_TRACKER_Y_OVERFLOW:     return "angle tracker Y overflow \n";
        default:                           return "unknown fault \n";
    }
}


void FaultManager::sendErrorMessage(FaultCode fault_code) {
    Serial.write(getMessage(fault_code));
}
