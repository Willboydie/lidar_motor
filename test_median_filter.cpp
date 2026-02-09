// Standalone test harness for MedianFilter - runs on host PC
// Compile: g++ -std=c++17 -o test_median_filter test_median_filter.cpp

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <chrono>

// Mock Arduino types
using String = std::string;

// Inline the filter for testing (copy of MedianFilter.hpp logic)
template <uint16_t WINDOW_SIZE, uint8_t BIT_RES>
class MedianFilter {
public:
    MedianFilter() { reset(); }

    void reset() {
        head = 0;
        count = 0;
        median = 0;
        ltMedian = 0;
        memset(histogram, 0, sizeof(histogram));
    }

    float operator()(int32_t new_val) {
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
            upper_val = median + 1;
            while (upper_val < HIST_SIZE - 1 && histogram[upper_val] == 0) {
                ++upper_val;
            }
        }

        return static_cast<float>(lower_val + upper_val) * 0.5f;
    }

    // Expose internals for debugging
    uint32_t getLtMedian() const { return ltMedian; }
    uint32_t getMedian() const { return median; }
    uint32_t getCount() const { return count; }
    static constexpr uint32_t getHistSize() { return HIST_SIZE; }

private:
    static constexpr uint32_t HIST_SIZE = 1 << BIT_RES;
    uint16_t window[WINDOW_SIZE];
    uint16_t histogram[HIST_SIZE];
    uint32_t head, count, median, ltMedian;
};

// Debug logging helper
std::ofstream logFile;

void logDebug(const char* hypothesisId, const char* location, const char* message, 
              int32_t val1 = 0, int32_t val2 = 0, int32_t val3 = 0) {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    
    logFile << "{\"hypothesisId\":\"" << hypothesisId << "\","
            << "\"location\":\"" << location << "\","
            << "\"message\":\"" << message << "\","
            << "\"data\":{\"val1\":" << val1 << ",\"val2\":" << val2 << ",\"val3\":" << val3 << "},"
            << "\"timestamp\":" << ms << "}\n";
    logFile.flush();
}

int main() {
    // #region agent log
    logFile.open("c:\\Users\\Will Boyd\\Projects\\lidar_motor\\.cursor\\debug.log", std::ios::app);
    if (!logFile.is_open()) {
        printf("ERROR: Could not open debug.log\n");
        return 1;
    }
    // #endregion

    printf("=== MedianFilter Bug Detection Tests ===\n\n");

    // Test A: Out-of-bounds histogram access
    // #region agent log
    logDebug("A", "test_oob", "Testing out-of-bounds: BIT_RES=8 means HIST_SIZE=256", 256, 0, 0);
    // #endregion
    {
        printf("Test A: Out-of-bounds histogram access\n");
        MedianFilter<5, 8> filter; // HIST_SIZE = 256
        
        // #region agent log
        logDebug("A", "test_oob:before", "About to insert value 300 into filter with HIST_SIZE=256", 300, 256, 0);
        // #endregion
        
        // This should cause OOB access if value >= 256
        filter(100);  // OK
        filter(150);  // OK
        float result = filter(300);  // BUG: 300 >= 256, OOB access!
        
        // #region agent log
        logDebug("A", "test_oob:after", "Result after OOB insert (undefined behavior may have occurred)", (int32_t)result, filter.getMedian(), filter.getLtMedian());
        // #endregion
        
        printf("  Inserted value 300 into filter with HIST_SIZE=256\n");
        printf("  Result: %.1f (undefined behavior - memory corruption likely)\n\n", result);
    }

    // Test B: Unsigned underflow of ltMedian
    // #region agent log
    logDebug("B", "test_underflow", "Testing ltMedian underflow scenario", 0, 0, 0);
    // #endregion
    {
        printf("Test B: ltMedian underflow\n");
        MedianFilter<3, 8> filter;
        
        // Fill window with values where median is high
        filter(200);
        filter(200);
        filter(200);
        
        // #region agent log
        logDebug("B", "test_underflow:before_remove", "State before removing value below median", 
                 filter.getLtMedian(), filter.getMedian(), filter.getCount());
        // #endregion
        
        // Now insert a lower value - old 200 gets removed
        // If ltMedian is 0 and old_val(200) < median, ltMedian-- causes underflow
        float result = filter(50);
        
        // #region agent log
        logDebug("B", "test_underflow:after", "State after potential underflow", 
                 filter.getLtMedian(), filter.getMedian(), (int32_t)result);
        // #endregion
        
        printf("  After [200,200,200] then inserting 50\n");
        printf("  ltMedian=%u, median=%u, result=%.1f\n", 
               filter.getLtMedian(), filter.getMedian(), result);
        if (filter.getLtMedian() > 1000000) {
            printf("  BUG CONFIRMED: ltMedian underflowed to %u!\n\n", filter.getLtMedian());
        } else {
            printf("  ltMedian appears normal\n\n");
        }
    }

    // Test C: Upper median off-by-one (can't reach last bin)
    // #region agent log
    logDebug("C", "test_offbyone", "Testing upper median at max bin value", 0, 0, 0);
    // #endregion
    {
        printf("Test C: Upper median off-by-one at max histogram bin\n");
        MedianFilter<4, 4> filter; // HIST_SIZE = 16, max valid value = 15
        
        // Create scenario where upper median should be at bin 15
        filter(15);
        filter(15);
        filter(14);
        float result = filter(14);
        
        // #region agent log
        logDebug("C", "test_offbyone:result", "Values [15,15,14,14] - expected median 14.5", 
                 (int32_t)(result * 10), filter.getMedian(), filter.getLtMedian());
        // #endregion
        
        printf("  Values: [15, 15, 14, 14]\n");
        printf("  Expected median: 14.5, Got: %.1f\n", result);
        if (result != 14.5f) {
            printf("  BUG CONFIRMED: Upper median search failed!\n\n");
        } else {
            printf("  Result correct\n\n");
        }
    }

    // Test D: Simple median correctness
    // #region agent log
    logDebug("D", "test_basic", "Testing basic median correctness", 0, 0, 0);
    // #endregion
    {
        printf("Test D: Basic median correctness\n");
        MedianFilter<5, 8> filter;
        
        filter(10);
        filter(20);
        filter(30);
        filter(40);
        float result = filter(50);
        
        // #region agent log
        logDebug("D", "test_basic:result", "Values [10,20,30,40,50] - expected median 30", 
                 (int32_t)result, filter.getMedian(), filter.getLtMedian());
        // #endregion
        
        printf("  Values: [10, 20, 30, 40, 50]\n");
        printf("  Expected median: 30, Got: %.1f\n\n", result);
    }

    // Test E: Even count median (should average two middle values)
    // #region agent log
    logDebug("E", "test_even", "Testing even count median averaging", 0, 0, 0);
    // #endregion
    {
        printf("Test E: Even count median averaging\n");
        MedianFilter<4, 8> filter;
        
        filter(10);
        filter(20);
        filter(30);
        float result = filter(40);
        
        // #region agent log
        logDebug("E", "test_even:result", "Values [10,20,30,40] - expected median 25", 
                 (int32_t)(result * 10), filter.getMedian(), filter.getLtMedian());
        // #endregion
        
        printf("  Values: [10, 20, 30, 40]\n");
        printf("  Expected median: 25 (avg of 20,30), Got: %.1f\n", result);
        if (result != 25.0f) {
            printf("  BUG: Median averaging incorrect!\n\n");
        } else {
            printf("  Result correct\n\n");
        }
    }

    // #region agent log
    logDebug("DONE", "main", "All tests completed", 0, 0, 0);
    logFile.close();
    // #endregion

    printf("=== Tests Complete - Check debug.log for detailed analysis ===\n");
    return 0;
}
