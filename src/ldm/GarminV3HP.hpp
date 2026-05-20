#pragma once

#include "LDM.hpp"

#define DEFAULT_ADDRESS 0x62
#define DEFAULT_DELAY 0
#define I2C_CLOCK_SPEED 1000000

// create a config struct for easy user selection

class GarminV3HP : public LDM {
public:
    GarminV3HP(int _scl, int _sda, int _mode_pin);
    void init() override;
    void reconfigure(int config_num) override;

private:
    void requestDistance() override;
    void readLatestDistance() override;
    void write(uint8_t reg, uint8_t* data_bytes, uint8_t length);
    void read(uint8_t reg, uint8_t* data_bytes, uint8_t length);
    void configure();
    void resetReferenceFilter();


    uint8_t address = DEFAULT_ADDRESS;
    uint8_t config_number = 0;

};