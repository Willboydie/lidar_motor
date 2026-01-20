#pragma once

#include "Sensor.hpp"
#include "Arduino.h"
#include "SPI.h"
#include "../utils/constants.h"
#include "../utils/tweakable_params.h"

#define ANGLE_REGISTER 0x3FFF
#define BIT_RESOLUTION 14
#define COMMAND_PARITY_BIT 15
#define COMMAND_RW_BIT 14
#define DATA_START_BIT 13
#define SPI_CLOCK_SPEED 10000000
#define SPI_MODE SPI_MODE1


class AS5048 : public Sensor {
public:
    AS5048(int cs);
    ~AS5048() override { deinit(); }
    bool init() override;
    bool init(SPIClass* _spi);
    void deinit() override;

private:
    byte spiCalcEvenParity(word v);
    uint16_t getRawCount() override;

    int cs_pin;
    SPIClass* spi;
    SPISettings spi_settings;
};