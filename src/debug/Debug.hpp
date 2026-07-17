#pragma once

#include <Arduino.h>
#include "../utils/constants.h"

constexpr uint8_t DEBUG_CMD = 0b11111111;
constexpr uint8_t DEBUG_HEADER[3] = {DEBUG_CMD, 0b0100, 0b0110}; // read 100 bytes

class Debug {
public:
    template<typename T>
    static void msg(const T& text) {
        if (mode) {
            Serial.write(DEBUG_HEADER, 3);
            Serial.println(text);
        }
    }
    static bool mode;
};