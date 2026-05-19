#include "AS5048.hpp"

constexpr float _2pi_over_cpr = _2PI / CPR;


AS5048::AS5048(int cs) : Sensor() {
    cs_pin = cs;
}

bool AS5048::init() {
    return init(&SPI);
}

bool AS5048::init(SPIClass* _spi) {
    spi = _spi;
    spi_settings = SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE);
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
#if defined(STM32G4xx)
    spi->setMISO(SENSOR_SPI_MISO);
    spi->setMOSI(SENSOR_SPI_MOSI);
    spi->setSCLK(SENSOR_SPI_SCK);
#endif
    spi->begin();
    initialised = true;
    return true;
}

float AS5048::getSensorAngle() {
    return static_cast<float>(getRawAngleCount()) * _2pi_over_cpr;
}

void AS5048::updateDiagnostics() {
    AS5048Diagnostics diagnostics = readDiagnostics();
    this->agc = diagnostics.agc;
    this->ocf = diagnostics.ocf;
    this->cof = diagnostics.cof;
    this->compLow = diagnostics.compLow;
    this->compHigh = diagnostics.compHigh;
    this->magnitude = getMagnitudeCount();
}

AS5048Diagnostics AS5048::readDiagnostics() {
	uint16_t command = AS5048A_DIAGNOSTICS_REG | AS5048A_RW; // set r=1, result ix 0x7FFD
	spi_write(command);
	AS5048Diagnostics result = {
			.reg = (nop()& AS5048A_RESULT_MASK)
	};
	return result;
}
 
uint16_t AS5048::getRawAngleCount() {
    uint16_t command = AS5048A_ANGLE_REG | AS5048A_PARITY | AS5048A_RW; // set r=1 and parity=1, result ix 0xFFFF
    spi_write(command);
    uint16_t angle = nop();
    return angle & AS5048A_RESULT_MASK;
}

uint16_t AS5048::getMagnitudeCount() {
    uint16_t command = AS5048A_MAGNITUDE_REG | AS5048A_RW; // set r=1, result ix 0x7FFE
    spi_write(command);
    uint16_t magnitude = nop();
    return magnitude & AS5048A_RESULT_MASK;
}

uint16_t AS5048::nop() {
    return spi_write(0xFFFF);
}

uint16_t AS5048::spi_write(uint16_t command) {
    //Send the command
    spi->beginTransaction(spi_settings);
    digitalWrite(cs_pin, LOW);
    uint16_t data = spi->transfer16(command);
    digitalWrite(cs_pin, HIGH);
    spi->endTransaction();
    // error_flag = data & AS5048A_ERRFLG;
    return data;
}


//---------------------------------------------------------------

byte AS5048::spiCalcEvenParity(word v) {  // unused in this implementation
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1;
}


//Closes the SPI connection
// SPI has an internal SPI-device counter, for each init()-call the deinit() function must be called exactly 1 time
void AS5048::deinit() {
    digitalWrite(cs_pin, HIGH); // deselect
    pinMode(cs_pin, INPUT);     // release the pin

    if (!spi) return;

    spi->end();
    spi = nullptr;
}
