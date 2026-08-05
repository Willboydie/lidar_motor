#pragma once

#include "Arduino.h"
#include <Wire.h>

class LDM {

public:
    LDM(int _scl, int _sda, int _mode_pin);
    virtual ~LDM() { disable(); }
    virtual void init() = 0;
    void enable();
    void disable();
    virtual void reconfigure(int config_num) = 0;
    
    // Interrupt-driven API
    void triggerMeasurement();                    // Start a new measurement
    bool isDataReady() { return dataReady; }      // Check if ready (non-blocking)
    void readDistance();                          // Read result (only call when ready)
    
    // returns the midpoint between the ready and trigger timestamps: our best estimate of the measurement timestamp
    // unsigned long getLatestTimestamp() { return (unsigned long)( (ready_timestamp + trigger_timestamp) / 2 ); }
    uint16_t getLatestDistance() { return latest_distance_cm; }
    bool isEnabled() { return enabled; }
    bool isInitialised() { return initialised; }

    // Called by ISR - must be public for the static wrapper
    void onMeasurementReady();

protected:
    volatile bool dataReady = false;     // Set by interrupt
    uint16_t latest_distance_cm;
    // unsigned long ready_timestamp;
    // unsigned long trigger_timestamp;

    int SCL;
    int SDA;
    int MODE_PIN;
    bool enabled = false;
    bool initialised = false;
    void markInitialised() { initialised = true; }

private:
    virtual void requestDistance() = 0;
    virtual void readLatestDistance() = 0;
};
