#pragma once

#include "Arduino.h"
#include <Wire.h>

#define I2C_CLOCK_SPEED 400000

class LDM {

public:
    LDM(int _scl, int _sda);
    virtual void init() = 0;
    void enable();
    void disable();
    void update();

    float getLatestTimestamp() { return latest_timestamp; }
    float getLatestDistance() { return latest_distance; }

private:
    virtual void requestDistance() = 0;
    virtual bool readSensor() = 0;

    float latest_timestamp = 0.0f;
    float latest_distance = 0.0f;

    int SCL;
    int SDA;

    bool enabled = false;

};