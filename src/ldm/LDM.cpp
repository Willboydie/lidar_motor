#include "LDM.hpp"
#include "../debug/Debug.hpp"
#include "../fault_manager/FaultManager.hpp"

// Global pointer for ISR to access the instance
static LDM* ldmInstance = nullptr;

// Static ISR wrapper
static void ldmISR() {
    if (ldmInstance) {
        ldmInstance->onMeasurementReady();
    }
}

LDM::LDM(int _scl, int _sda, int _mode_pin) : SCL(_scl), SDA(_sda), MODE_PIN(_mode_pin) {}

void LDM::enable() {
    if (enabled) return;
    if (!initialised) {
        faultManager.addFault(FAULT_SYS_NOT_INITIALISED);
        return;
    }
    enabled = true;
    ldmInstance = this;
    attachInterrupt(digitalPinToInterrupt(MODE_PIN), ldmISR, RISING);
    dataReady = false;  // Reset state when re-enabling
    triggerMeasurement();  // Trigger initial measurement
    Debug::msg("LDM ENABLED");
}

void LDM::disable() {
    if (!enabled) return;
    detachInterrupt(digitalPinToInterrupt(MODE_PIN));
    ldmInstance = nullptr;
    enabled = false;
    dataReady = false;
    Debug::msg("LDM DISABLED");
}

void LDM::onMeasurementReady() {
    dataReady = true;
}

void LDM::triggerMeasurement() {
    if (!enabled) return;
    dataReady = false;
    requestDistance();
}

void LDM::readDistance() {
    readLatestDistance();
}