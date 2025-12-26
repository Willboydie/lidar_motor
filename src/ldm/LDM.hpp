#pragma once

#include "Arduino.h"
#include <Wire.h>

#define I2C_CLOCK_SPEED 400000

class LDM {

public:
    LDM(int _scl, int _sda, int _mode_pin);
    virtual void init() = 0;
    void enable();
    void disable();
    
    // Interrupt-driven API
    void triggerMeasurement();                    // Start a new measurement
    bool isDataReady() { return dataReady; }      // Check if ready (non-blocking)
    bool readDistance();                          // Read result (only call when ready)
    
    // Legacy polling API (kept for compatibility)
    bool readSensor();

    unsigned long getLatestTimestamp() { return latest_timestamp; }
    uint16_t getLatestDistance() { return latest_distance_cm; }
    bool isEnabled() { return enabled; }

    // Called by ISR - must be public for the static wrapper
    void onMeasurementReady();

protected:
    volatile bool dataReady = false;     // Set by interrupt
    bool busy = true;                    // For legacy polling mode
    uint16_t latest_distance_cm;
    unsigned long latest_timestamp;
    int MODE_PIN;

private:
    virtual void requestDistance() = 0;
    virtual void readLatestDistance() = 0;
    virtual void pollStatus() = 0;

    int SCL;
    int SDA;

    bool enabled = false;
};
