#include "Parser.hpp"
#include "../interface/Interface.hpp"
#include "../fault_manager/FaultManager.hpp"
#include <cstring>

Parser::Parser(Interface& interface, uint8_t* _payload_buffer, uint32_t _payload_max_len)
    : interface(interface)
    , payloadBuffer(_payload_buffer)
    , payload_max_len(_payload_max_len) {}

void Parser::reset() {
    state = State::AWAIT_CMD;
    // Only clear bytes we own — `length` may exceed payload_max_len on oversize frames.
    if (payloadBuffer != nullptr && payload_max_len > 0) {
        memset(payloadBuffer, 0, payload_max_len);
    }
    payloadBytesRead = 0;
    oversize = false;
    length = 0;
    cmd = 0;
}

void Parser::pump(ByteIO& io) {
    while (io.available() > 0) {
        switch (state) {
            case State::AWAIT_CMD:
                cmd = static_cast<uint8_t>(io.read());
                payloadBytesRead = 0;
                oversize = false;
                length = 0;
                state = State::AWAIT_LEN_LO;
                break;

            case State::AWAIT_LEN_LO:
                length = static_cast<uint8_t>(io.read());
                state = State::AWAIT_LEN_HI;
                break;

            case State::AWAIT_LEN_HI:
                length |= static_cast<uint32_t>(static_cast<uint8_t>(io.read())) << 8;

                if (length > payload_max_len) {
                    oversize = true;
                    payloadBytesRead = 0;
                    state = State::AWAIT_PAYLOAD;
                } else if (length == 0) {
                    interface.onCommand(cmd, nullptr, 0);
                    state = State::AWAIT_CMD;
                } else {
                    payloadBytesRead = 0;
                    state = State::AWAIT_PAYLOAD;
                }
                break;

            case State::AWAIT_PAYLOAD: {
                if (oversize) {
                    (void)io.read();
                    payloadBytesRead++;
                    if (payloadBytesRead >= length) {
                        faultManager.addFault(FAULT_PROTO_OVERFLOW);
                        reset();
                    }
                    break;
                }

                payloadBuffer[payloadBytesRead++] = static_cast<uint8_t>(io.read());

                if (payloadBytesRead >= length) {
                    interface.onCommand(cmd, payloadBuffer, length);
                    reset();
                }
                break;
            }
        }
    }
}
