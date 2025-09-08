#include "mutex_wrapper.h"

// =============================================================================
// MUTEX WRAPPER IMPLEMENTATION
// =============================================================================

MutexWrapper::MutexWrapper(const char* name, TickType_t defaultTimeout)
    : name(name), defaultTimeout(defaultTimeout), timeoutCount(0), takeCount(0) {
    mutex = xSemaphoreCreateMutex();
}

MutexWrapper::~MutexWrapper() {
    if (mutex != nullptr) {
        vSemaphoreDelete(mutex);
    }
}

bool MutexWrapper::take(TickType_t timeout) {
    if (mutex == nullptr) return false;
    
    TickType_t actualTimeout = (timeout == 0) ? defaultTimeout : timeout;
    bool result = xSemaphoreTake(mutex, actualTimeout) == pdTRUE;
    
    if (result) {
        takeCount++;
    } else {
        timeoutCount++;
    }
    
    return result;
}

void MutexWrapper::give() {
    if (mutex != nullptr) {
        xSemaphoreGive(mutex);
    }
}

bool MutexWrapper::isTaken() {
    // This is a simplified check - in practice, you might want more sophisticated logic
    return xSemaphoreTake(mutex, 0) == pdTRUE;
}

void MutexWrapper::resetStatistics() {
    timeoutCount = 0;
    takeCount = 0;
}

// =============================================================================
// LOCK GUARD IMPLEMENTATION
// =============================================================================

MutexWrapper::LockGuard::LockGuard(MutexWrapper& mutex, TickType_t timeout)
    : mutex(mutex), locked(false) {
    locked = mutex.take(timeout);
}

MutexWrapper::LockGuard::~LockGuard() {
    if (locked) {
        mutex.give();
    }
}
