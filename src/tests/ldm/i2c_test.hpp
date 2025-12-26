#include <Wire.h>

constexpr uint8_t kI2CStart = 0x03;   // Skip reserved
constexpr uint8_t kI2CEnd   = 0x77;   // 7-bit max
constexpr uint32_t kClockHz = 400000; // Match your lidar config
constexpr int kSclPin = 18;           // adjust for your board (used on ESP32/ESP8266)
constexpr int kSdaPin = 19;           // adjust for your board (used on ESP32/ESP8266)

void i2c_test() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // On ESP32/ESP8266 we can specify pins; on AVR and many others use default pins
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  Wire.begin(kSdaPin, kSclPin);
#else
  Wire.begin();
#endif
  Wire.setClock(kClockHz);

  Serial.println("I2C scan starting...");

  uint8_t found = 0;
  for (uint8_t addr = kI2CStart; addr <= kI2CEnd; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
      ++found;
    } else if (err == 4) {
      Serial.print("Unknown error at 0x");
      Serial.println(addr, HEX);
    }
    delay(2); // small pause between probes
  }

  if (found == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Total devices found: ");
    Serial.println(found);
  }
}
