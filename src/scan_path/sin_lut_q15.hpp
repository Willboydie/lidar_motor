#pragma once
#include <cstdint>

constexpr uint32_t SIN_LUT_BITS = 11;
constexpr uint32_t SIN_LUT_SIZE = (1 << SIN_LUT_BITS);

// Quarter-wave sine table, Q15 format
extern const int16_t sin_lut_q15[SIN_LUT_SIZE + 1];

// Fast fixed-point sine
int16_t sin_q15_u16(uint16_t phase);

// industry standard 32 bit phase accumulator
int16_t sin_q15_u32(uint32_t phase);
