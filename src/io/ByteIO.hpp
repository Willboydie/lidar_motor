#pragma once

#include <stdint.h>
#include <stddef.h>

// Minimal byte stream abstraction so the serial parser can be driven by a real
// UART on the firmware and by an in-memory fake in native unit tests.
class ByteIO {
public:
    virtual ~ByteIO() = default;

    // Number of bytes available to read right now.
    virtual uint32_t available() = 0;

    // Read one byte, or -1 if none are available.
    virtual int read() = 0;

    // Write len bytes; returns the number actually written.
    virtual uint32_t write(const uint8_t* data, uint32_t len) = 0;

    // Block until queued outbound bytes are flushed (no-op for fakes).
    virtual void flush() {}
};
