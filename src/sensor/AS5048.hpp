#pragma once

#include "Sensor.hpp"
#include "Arduino.h"
#include "SPI.h"
#include "../utils/constants.h"

#define ANGLE_REGISTER 0x3FFF
#define BIT_RESOLUTION 14
#define COMMAND_PARITY_BIT 15
#define COMMAND_RW_BIT 14
#define DATA_START_BIT 13
#define CPR (1 << BIT_RESOLUTION) // 14 bit resolution
#define SPI_CLOCK_SPEED 10000000
#define SPI_MODE SPI_MODE1


class AS5048 : public Sensor {
public:
    AS5048(int cs);
    bool init() override;
    bool init(SPIClass* _spi);
    void deinit() override;

private:
    float getSensorAngle() override;
    byte spiCalcEvenParity(word v);
    int getRawCount();

    int cs_pin;
    SPIClass* spi;
    SPISettings spi_settings;
};