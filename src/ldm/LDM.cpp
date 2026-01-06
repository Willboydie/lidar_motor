#include "LDM.hpp"

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
    enabled = true;
    ldmInstance = this;
    attachInterrupt(digitalPinToInterrupt(MODE_PIN), ldmISR, RISING);
}

void LDM::disable() {
    if (!enabled) return;
    detachInterrupt(digitalPinToInterrupt(MODE_PIN));
    ldmInstance = nullptr;
    enabled = false;
}

void LDM::onMeasurementReady() {
    dataReady = true;
    // ready_timestamp = micros();  // Capture time immediately in ISR
}

void LDM::triggerMeasurement() {
    if (!enabled) return;
    dataReady = false;
    requestDistance();
    // trigger_timestamp = micros();
}

bool LDM::readDistance() {
    readLatestDistance();
    return true;
}