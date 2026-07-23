#pragma once

#include "../angle_tracker/AngleTracker.hpp"


class ScanBuffer {
public:
    ScanBuffer::ScanBuffer(AngleTracker& _angle_tracker, uint8_t* _buffer, uint32_t _size);
    ~ScanBuffer() = default;

private:
    AngleTracker& angle_tracker;

    uint32_t scan_idx;
    float angleX;
    float angleY;

    uint8_t* buffer;
    uint32_t size = ;
    uint32_t buf_idx;
    bool hit = false;
};