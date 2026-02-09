#include "Jitter.hpp"

Jitter::Jitter(uint16_t _amplitude, uint16_t _period)
: amplitude(_amplitude)
, period(_period < 1 ? 1 : _period)
, increment(0)
{}

uint16_t Jitter::operator()(uint16_t reading) {
    bool positive = (increment++ / period) % 2 == 0;
    return reading + (positive ? amplitude : -amplitude);
}