#pragma once

#include <Arduino.h>
#include "ByteIO.hpp"

// ByteIO backed by the Arduino USB CDC Serial port (firmware only).
class SerialIO : public ByteIO {
public:
    explicit SerialIO(Stream& port) : port(port) {}

    uint32_t available() override {
        int n = port.available();
        return n > 0 ? static_cast<uint32_t>(n) : 0;
    }

    int read() override {
        return port.read();
    }

    uint32_t write(const uint8_t* data, uint32_t len) override {
        return static_cast<uint32_t>(port.write(data, len));
    }

    void flush() override {
        port.flush();
    }

private:
    Stream& port;
};
