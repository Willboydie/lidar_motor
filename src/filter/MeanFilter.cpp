#include "MeanFilter.hpp"

MeanFilter::MeanFilter() 
: head(0)
, int_sum(0)
, float_sum(0.0f)
{}

int32_t MeanFilter::operator()(int32_t reading) {
    int_sum += reading - int_buffer[head];
    int_buffer[head] = reading;
    head = (head + 1) % window_size;

    // .5-and-up rounding
    return (int_sum + window_size / 2) / window_size;
}


float MeanFilter::operator()(float reading) {
    float_sum += reading - float_buffer[head];
    float_buffer[head] = reading;
    head = (head + 1) % window_size;
    return float_sum * one_over_window_size;
}

void MeanFilter::reset() {
    int_sum = 0;
    float_sum = 0.0f;
    head = 0;
    for (uint16_t i = 0; i < window_size; i++) {
        int_buffer[i] = 0;
        float_buffer[i] = 0.0f;
    }
}