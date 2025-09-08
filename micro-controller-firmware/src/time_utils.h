#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <stdint.h>

// =============================================================================
// COMMON TIME UTILITIES
// =============================================================================

namespace TimeUtils {
    // Get current timestamp in microseconds
    inline uint32_t getCurrentTimestampUs() {
        return micros();
    }
    
    // Get current timestamp in milliseconds
    inline uint32_t getCurrentTimestampMs() {
        return millis();
    }
    
    // Convert microseconds to milliseconds
    inline uint32_t usToMs(uint32_t microseconds) {
        return microseconds / 1000;
    }
    
    // Convert milliseconds to microseconds
    inline uint32_t msToUs(uint32_t milliseconds) {
        return milliseconds * 1000;
    }
    
    // Check if a timeout has expired
    inline bool isTimeoutExpired(uint32_t startTime, uint32_t timeoutMs) {
        return (getCurrentTimestampMs() - startTime) >= timeoutMs;
    }
    
    // Get elapsed time since start time
    inline uint32_t getElapsedTimeMs(uint32_t startTime) {
        return getCurrentTimestampMs() - startTime;
    }
    
    // Get elapsed time since start time in microseconds
    inline uint32_t getElapsedTimeUs(uint32_t startTime) {
        return getCurrentTimestampUs() - startTime;
    }
}

// =============================================================================
// CONVENIENCE MACROS
// =============================================================================

#define GET_TIMESTAMP_US() TimeUtils::getCurrentTimestampUs()
#define GET_TIMESTAMP_MS() TimeUtils::getCurrentTimestampMs()

#endif // TIME_UTILS_H
