#pragma once

#include <Arduino.h>


template <uint16_t WINDOW_SIZE, uint8_t BIT_RES>
class MedianFilter {
public:
    MedianFilter() {
        reset();
    }

    void reset() {
        head = 0;
        count = 0;
        median = 0;
        ltMedian = 0;
        memset(histogram, 0, sizeof(histogram));
    }

    float operator()(int32_t new_val) {

        if (new_val < 0 || new_val >= HIST_SIZE) return -123456.0f;

        if (count == 0) {
            window[0] = new_val;
            histogram[new_val] = 1;
            median = new_val;
            head = 1;
            count = 1;
            return (float)median;
        }

        // 1. Remove oldest
        if (count == WINDOW_SIZE) {
            uint16_t old_val = window[head];
            histogram[old_val]--;
            if (old_val < median) ltMedian--;
        } else {
            count++;
        }

        // 2. Add newest
        window[head] = new_val;
        histogram[new_val]++;
        if (new_val < median) ltMedian++;
        
        if (++head >= WINDOW_SIZE) head = 0;

        // 3. Rebalance to find lower median at position (count-1)/2
        const uint32_t target_lower = (count - 1) >> 1;

        while (ltMedian > target_lower && median > 0) {
            --median;
            ltMedian -= histogram[median];
        }
        while (ltMedian + histogram[median] <= target_lower && median < HIST_SIZE - 1) {
            ltMedian += histogram[median];
            ++median;
        }

        // 4. Find upper median at position count/2
        uint32_t lower_val = median;
        uint32_t upper_val = median;
        
        const uint32_t target_upper = count >> 1;
        uint32_t cumulative = ltMedian + histogram[median];
        
        if (cumulative <= target_upper) {
            // Upper median is in a higher bin
            upper_val = median + 1;
            while (upper_val < HIST_SIZE - 1 && histogram[upper_val] == 0) {
                ++upper_val;
            }
        }

        return static_cast<float>(lower_val + upper_val) * 0.5f;
    }

private:
    static constexpr uint32_t HIST_SIZE = 1 << BIT_RES;
    uint16_t window[WINDOW_SIZE];
    uint16_t histogram[HIST_SIZE]; // Increased to uint16_t to allow windows > 255
    uint32_t head, count, median, ltMedian;
};