#pragma once
#include <cstdint>

constexpr int SIN_LUT_SIZE = (1 << 11);

// Quarter-wave sine table, Q15 format
extern const int16_t sin_lut_q15[SIN_LUT_SIZE + 1];

// Fast fixed-point sine
int16_t sin_q15_u16(uint16_t phase);
