#pragma once

#include <stdint.h>
#include <stddef.h>
#include "../io/ByteIO.hpp"

class Interface;

// Pure, platform-independent parser for the wire framing:
//
//   [cmd: 1][len_lo: 1][len_hi: 1][payload: len bytes]
//
// The parser holds no I/O of its own: call pump(io) whenever bytes may be
// available; it consumes only what is currently readable and returns.
class Parser {
public:
    Parser(Interface& interface, uint8_t* _payload_buffer, uint32_t _payload_max_len);

    // Consume all currently-available bytes from io (subject to backpressure).
    void pump(ByteIO& io);

    // Abandon any partially-parsed frame and return to waiting for a command.
    void reset();

private:
    Interface& interface;
    uint8_t* payloadBuffer;
    uint32_t payload_max_len;

    enum class State {
        AWAIT_CMD,
        AWAIT_LEN_LO,
        AWAIT_LEN_HI,
        AWAIT_PAYLOAD
    };

    State state = State::AWAIT_CMD;
    uint8_t cmd = 0;
    uint32_t length = 0;
    uint32_t payloadBytesRead = 0;
    bool oversize = false;
};
