#pragma once

#include "Sensor.hpp"
#include "Arduino.h"
#include "SPI.h"
#include "../utils/constants.h"
#include "../utils/tweakable_params.h"

#define SPI_CLOCK_SPEED 10000000
#define SPI_MODE SPI_MODE1

#define AS5048A_CPR 16384
#define AS5048A_ANGLE_REG 0x3FFF
#define AS5048A_ERROR_REG 0x0001
#define AS5048A_PROGCTL_REG 0x0003
#define AS5048A_OTPHIGH_REG 0x0016
#define AS5048A_OTPLOW_REG 0x0017
#define AS5048A_DIAGNOSTICS_REG 0x3FFD
#define AS5048A_MAGNITUDE_REG 0x3FFE
#define AS5048A_PARITY 0x8000
#define AS5048A_RW 0x4000
#define AS5048A_ERRFLG 0x4000
#define AS5048A_RESULT_MASK 0x3FFF

union AS5048Diagnostics {
	struct {
		uint16_t agc:8;
		uint16_t ocf:1;
		uint16_t cof:1;
		uint16_t compLow:1;
		uint16_t compHigh:1;
	};
	uint16_t reg;
};


class AS5048 : public Sensor {
public:
    AS5048(int cs);
    ~AS5048() override { deinit(); }
    using Sensor::init;
    bool hardwareInit() override { return init(&SPI); }
    bool init(SPIClass* _spi);
    void hardwareDeinit() override;
    float getSensorAngle() override;
    void updateDiagnostics() override;

private:
    uint16_t getRawAngleCount();
    uint16_t getMagnitudeCount();
    AS5048Diagnostics readDiagnostics();
    uint16_t nop();
    uint16_t spi_write(uint16_t command);
    byte spiCalcEvenParity(word v);

    int cs_pin;
    SPIClass* spi = nullptr;  // null until init(); deinit() guard relies on this
    SPISettings spi_settings;
};