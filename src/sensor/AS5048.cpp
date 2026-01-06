#include "AS5048.hpp"


AS5048::AS5048(int cs) {
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
    spi->begin();
    initialised = true;
    return true;
}

 
uint16_t AS5048::getRawCount() {
    
    word command = ANGLE_REGISTER;

    if (COMMAND_RW_BIT > 0) {
        command = ANGLE_REGISTER | (1 << COMMAND_RW_BIT);
    }

    if (COMMAND_PARITY_BIT > 0) {
            //Add a parity bit on the the MSB
        command |= ((word)spiCalcEvenParity(command) << COMMAND_PARITY_BIT);
    }

    //SPI - begin transaction
    spi->beginTransaction(spi_settings);

    //Send the command
    digitalWriteFast(cs_pin, LOW);
    spi->transfer16(command);
    digitalWriteFast(cs_pin,HIGH);

    delayMicroseconds(1); // delay 1us, the minimum time possible in plain arduino. 350ns is the required time for AMS sensors, 80ns for MA730, MA702

    //Now read the response
    digitalWriteFast(cs_pin, LOW);
    word register_value = spi->transfer16(0x00);
    digitalWriteFast(cs_pin, HIGH);

    //SPI - end transaction
    spi->endTransaction();

    register_value = register_value >> (1 + DATA_START_BIT - BIT_RESOLUTION);  //this should shift data to the rightmost bits of the word

    const static word data_mask = 0xFFFF >> (16 - BIT_RESOLUTION);

    return (uint16_t)(register_value & data_mask);  // Return the data, stripping the non data (e.g parity) bits
}


byte AS5048::spiCalcEvenParity(word v) {
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
