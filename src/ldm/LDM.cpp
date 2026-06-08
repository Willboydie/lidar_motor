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
    if (!initialised) {
        Serial.println("Can't enable LDM - not initialised");
        return;
    }
    enabled = true;
    ldmInstance = this;
    attachInterrupt(digitalPinToInterrupt(MODE_PIN), ldmISR, RISING);
    dataReady = false;  // Reset state when re-enabling
    triggerMeasurement();  // Trigger initial measurement
}

void LDM::disable() {
    if (!enabled) return;
    detachInterrupt(digitalPinToInterrupt(MODE_PIN));
    ldmInstance = nullptr;
    enabled = false;
}

void LDM::onMeasurementReady() {
    dataReady = true;
}

void LDM::triggerMeasurement() {
    if (!enabled) return;
    dataReady = false;
    requestDistance();
}

bool LDM::readDistance() {
    readLatestDistance();
    return true;
}