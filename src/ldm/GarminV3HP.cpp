#include "GarminV3HP.hpp"

GarminV3HP::GarminV3HP(int _scl, int _sda, int _mode_pin) : LDM(_scl, _sda, _mode_pin) {}

void GarminV3HP::init() {
    // Route the I2C peripheral to the SCL/SDA pins passed to the constructor.
    // On stm32duino these must be set before Wire.begin(); on Teensy the calls
    // are a no-op when the pins already match the default Wire instance.
    Wire.setSDA(SDA);
    Wire.setSCL(SCL);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);

    // Configure MODE pin as input for interrupt
    pinMode(MODE_PIN, INPUT);
    
    config_number = 0; // Default mode, balanced performance
    configure();
    enable();
    resetReferenceFilter();
    disable();
    Serial.println("GarminV3HP initialised with interrupt mode.");
}

/*------------------------------------------------------------------------------
  Configure Parameters
    0: Default mode, balanced performance.
    1: Short range, high speed. Uses 0x1d maximum acquisition count.
    2: Default range, higher speed short range. Turns on quick termination
        detection for faster measurements at short range (with decreased
        accuracy)
    3: Maximum range. Uses 0xff maximum acquisition count.
    4: High sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for high sensitivity and noise.
    5: Low sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for low sensitivity and noise.
    
  Note: Bit 0 of acqConfigReg is always set to enable MODE pin as status output
  for interrupt-driven operation. MODE pin goes LOW when busy, HIGH when ready.
------------------------------------------------------------------------------*/
void GarminV3HP::configure() {
    uint8_t sigCountMax;
    uint8_t acqConfigReg;
    uint8_t refCountMax;
    uint8_t thresholdBypass;

    switch (config_number) {
        case 0: // Default mode, balanced performance
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;

        case 1: // Short range, high speed
            sigCountMax     = 0x1d;
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x03;
            thresholdBypass = 0x00; // Default
            break;

        case 2: // Default range, higher speed short range
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x00;
            refCountMax     = 0x03;
            thresholdBypass = 0x00; // Default
            break;

        case 3: // Maximum range
            sigCountMax     = 0xff;
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;

        case 4: // High sensitivity detection, high erroneous measurements
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x80;
            break;

        case 5: // Low sensitivity detection, low erroneous measurements
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0xb0;
            break;

        case 6: // Short range, high speed, higher error
            sigCountMax     = 0x04;
            acqConfigReg    = 0x00; // turn off short_sig
            refCountMax     = 0x03;
            thresholdBypass = 0x00;
            break;
    }

    // Set bit 0 to enable MODE pin as status output (LOW=busy, HIGH=ready)
    acqConfigReg |= 0x01;

    write(0x02, &sigCountMax    , 1);
    write(0x04, &acqConfigReg   , 1);
    write(0x12, &refCountMax    , 1);
    write(0x1c, &thresholdBypass, 1);
}


void GarminV3HP::reconfigure(int config_num) {
    if (enabled) {
        Serial.println("Disable the Garmin first to change the configuration.");
        return;
    }
    config_number = config_num;
    configure();
    Serial.println("Garmin V3HP configured to preset " + String(config_number));
}



void GarminV3HP::requestDistance() {
    uint8_t data_byte = 0x01;
    write(0x00, &data_byte, 1);
}


void GarminV3HP::readLatestDistance() {
    uint8_t distance_data[2];
    read(0x0f, distance_data, 2);
    // Serial.println("Distance data: " + String((distance_data[0] << 8) | distance_data[1]));
    latest_distance_cm = (distance_data[0] << 8) | distance_data[1];
}



void GarminV3HP::write(uint8_t reg, uint8_t* data_bytes, uint8_t length) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(data_bytes, length);
    if ( Wire.endTransmission() ) Serial.println("> nack");
    delayMicroseconds(DEFAULT_DELAY);
}




void GarminV3HP::read(uint8_t reg,  uint8_t * data_bytes, uint8_t length) {
    // Portable two-step read (write register address with repeated start, then
    // read length bytes). The Teensy-only 8-arg Wire.requestFrom overload is
    // not available on stm32duino, so we always go via this standard path.
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false)) {  // repeated start
        Serial.println("> nack");
    }
    Wire.requestFrom(address, length, (uint8_t)true);

    uint8_t numHere = Wire.available();
    uint8_t i = 0;
    while (i < numHere) {
        data_bytes[i] = Wire.read(); i++;
    }

    delayMicroseconds(DEFAULT_DELAY);
}




void GarminV3HP::resetReferenceFilter() {
    uint8_t data_bytes[2];
    uint8_t acqConfigReg;
    uint8_t refCountMax;

    // Set bit 4 of the acquisition configuration register (disable reference filter)
    read(0x04, data_bytes, 1);  // Read address 0x04 (acquisition config register)
    acqConfigReg = data_bytes[0];                 // store for later restoration
    data_bytes[0] = data_bytes[0] | 0x10;          // turn on disable of ref filter
    write(0x04, data_bytes, 1); // write it back

    // Set reference integration count to max
    read(0x12, data_bytes, 1);  // Read address 0x12 (ref integration count)
    refCountMax = data_bytes[0];                  // store for later restoration
    data_bytes[0] = 0xff;                         // we want to reference to overflow quickly
    write(0x12, data_bytes, 1); // write ref integration count

    // Trigger a measurement
    requestDistance();
    while ( !isDataReady() ) {
        delay(100);
    }
    // ... no need to read the distance, it is immaterial

    // Restore previous reference integration count
    data_bytes[0] = refCountMax;
    write(0x12, data_bytes, 1);

    // Restore previous acquisition configuration register (re-enabling reference filter)
    data_bytes[0] = acqConfigReg;
    write(0x04, data_bytes, 1);
}