#pragma once

#include <Arduino.h>
#include <cstring>
#include "../utils/constants.h"
#include "../utils/cmd_defs.h"

// HOST_PROTOCOL.md §7 — one length-prefixed frame of raw bytes (no NUL, no \r\n):
//   [header : uint8 = 0xFF] [len : uint16 LE] [payload : len bytes of ASCII]
//
// Pack the whole frame into a uint8_t buffer and write it once. Do not write a
// uint8+uint16 struct without pack: that object is 4 bytes, and the padding
// byte is then interpreted by the host as part of `len`.
#pragma pack(push, 1)
struct DebugPacket {
    uint8_t header;
    uint16_t len;
};
#pragma pack(pop)
static_assert(sizeof(DebugPacket) == 3, "DebugPacket prefix must be 3 bytes");

class Debug {
public:
    static constexpr uint16_t kMaxLen = 256;

    static void msg(const char* text) {
        if (!mode || text == nullptr) {
            return;
        }
        uint16_t n = static_cast<uint16_t>(strlen(text));
        if (n > kMaxLen) {
            n = kMaxLen;
        }
        uint8_t packet[3 + kMaxLen];
        packet[0] = DEBUG_PACKET_HEADER;
        packet[1] = static_cast<uint8_t>(n);
        packet[2] = static_cast<uint8_t>(n >> 8);
        for (uint16_t i = 0; i < n; ++i) {
            packet[3 + i] = static_cast<uint8_t>(text[i]);
        }
        Serial.write(packet, static_cast<size_t>(3) + n);
    }
    static bool mode;
    static void setMode(bool _mode) {
        mode = _mode;
    }
};
