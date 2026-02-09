#include "ModeFilter.hpp"

ModeFilter::ModeFilter(uint32_t _size)
: window_mask(_size - 1)
{
    reset();
}

uint16_t ModeFilter::operator()(uint16_t new_val) {
    // 1. Extract old value
    uint16_t old_val = history[head];
    history[head] = new_val;
    head = (head + 1) & window_mask;

    // 2. Process Outgoing
    uint8_t old_freq = counts[old_val];
    if (old_freq > 0) {
        freq_bin[old_freq]--;
        counts[old_val]--;
        if (old_freq > 1) {
            freq_bin[old_freq - 1]++;
        }
        
        // If the outgoing value was the ONLY mode, reduce max_freq
        if (old_freq == max_freq && freq_bin[old_freq] == 0) max_freq--;
    }

    // 3. Process Incoming
    uint8_t old_freq_new = counts[new_val];  // Get frequency before increment
    uint8_t new_freq = ++counts[new_val];
    
    // Only update freq_bin if the value was already in the window
    if (old_freq_new > 0) {
        freq_bin[old_freq_new]--;
    }
    freq_bin[new_freq]++;

    // 4. Update Mode
    if (new_freq >= max_freq) {
        max_freq = new_freq;
        current_mode = new_val;
    }

    return current_mode;
}

void ModeFilter::reset() {
    memset(counts, 0, sizeof(counts));
    memset(freq_bin, 0, sizeof(freq_bin));
    memset(history, 0, sizeof(history));
    head = 0;
    current_mode = 0;
    max_freq = 0;
}