#ifndef MUTEX_WRAPPER_H
#define MUTEX_WRAPPER_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// =============================================================================
// COMMON MUTEX WRAPPER CLASS
// =============================================================================

class MutexWrapper {
public:
    MutexWrapper(const char* name = nullptr, TickType_t defaultTimeout = pdMS_TO_TICKS(10));
    ~MutexWrapper();
    
    // Core mutex operations
    bool take(TickType_t timeout = 0);  // 0 = use default timeout
    void give();
    bool isTaken();
    
    // RAII-style lock guard
    class LockGuard {
    public:
        LockGuard(MutexWrapper& mutex, TickType_t timeout = 0);
        ~LockGuard();
        bool isLocked() const { return locked; }
    private:
        MutexWrapper& mutex;
        bool locked;
    };
    
    // Get mutex handle for direct FreeRTOS operations if needed
    SemaphoreHandle_t getHandle() const { return mutex; }
    
    // Statistics
    uint32_t getTimeoutCount() const { return timeoutCount; }
    uint32_t getTakeCount() const { return takeCount; }
    void resetStatistics();
    
private:
    SemaphoreHandle_t mutex;
    const char* name;
    TickType_t defaultTimeout;
    volatile uint32_t timeoutCount;
    volatile uint32_t takeCount;
};

// =============================================================================
// CONVENIENCE MACROS
// =============================================================================

#define MUTEX_LOCK(mutex) MutexWrapper::LockGuard _lock(mutex)
#define MUTEX_LOCK_TIMEOUT(mutex, timeout) MutexWrapper::LockGuard _lock(mutex, timeout)

// =============================================================================
// SPECIALIZED MUTEX WRAPPERS
// =============================================================================

class SpiMutex : public MutexWrapper {
public:
    SpiMutex() : MutexWrapper("SPI", pdMS_TO_TICKS(5)) {}
};

class LogMutex : public MutexWrapper {
public:
    LogMutex() : MutexWrapper("LOG", pdMS_TO_TICKS(10)) {}
};

class ErrorMutex : public MutexWrapper {
public:
    ErrorMutex() : MutexWrapper("ERROR", pdMS_TO_TICKS(10)) {}
};

#endif // MUTEX_WRAPPER_H
