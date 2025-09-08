#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"

// =============================================================================
// COMMON ERROR HANDLER BASE CLASS
// =============================================================================

class ErrorHandler {
public:
    ErrorHandler(const char* componentName);
    virtual ~ErrorHandler() = default;
    
    // Error handling
    void handleError(const char* errorMessage, uint8_t errorFlag = 0);
    void handleErrorf(const char* format, ...);
    
    // Error statistics
    uint32_t getErrorCount() const { return errorCount; }
    uint32_t getLastErrorTime() const { return lastErrorTime; }
    bool hasRecentErrors(uint32_t timeWindowMs = 1000) const;
    
    // Health checking
    bool isHealthy() const;
    void resetErrorCount();
    
    // Error rate monitoring
    float getErrorRate() const; // errors per second
    bool isErrorRateAcceptable(float maxRate = 1.0f) const;
    
protected:
    const char* componentName;
    volatile uint32_t errorCount;
    volatile uint32_t lastErrorTime;
    uint32_t firstErrorTime;
    
    // Virtual method for custom error handling
    virtual void onError(const char* errorMessage, uint8_t errorFlag);
};

// =============================================================================
// TASK-SPECIFIC ERROR HANDLER
// =============================================================================

class TaskErrorHandler : public ErrorHandler {
public:
    TaskErrorHandler(const char* componentName);
    
    // Task-specific error handling
    void handleTaskError(const char* errorMessage, uint8_t errorFlag = 0);
    void handleDeadlineMiss();
    void handleMutexTimeout();
    void handleQueueOverflow();
    
    // Task health checking
    bool isTaskHealthy() const;
    uint32_t getMissedDeadlines() const { return missedDeadlines; }
    
protected:
    void onError(const char* errorMessage, uint8_t errorFlag) override;
    
private:
    uint32_t missedDeadlines;
    uint32_t mutexTimeouts;
    uint32_t queueOverflows;
};

// =============================================================================
// COMMUNICATION ERROR HANDLER
// =============================================================================

class CommunicationErrorHandler : public ErrorHandler {
public:
    CommunicationErrorHandler(const char* componentName);
    
    // Communication-specific error handling
    void handleConnectionDrop();
    void handleMessageSendFailure();
    void handleMessageReceiveFailure();
    void handleTimeout();
    
    // Communication health checking
    bool isCommunicationHealthy() const;
    uint32_t getConnectionDrops() const { return connectionDrops; }
    uint32_t getSendFailures() const { return sendFailures; }
    uint32_t getReceiveFailures() const { return receiveFailures; }
    
protected:
    void onError(const char* errorMessage, uint8_t errorFlag) override;
    
private:
    uint32_t connectionDrops;
    uint32_t sendFailures;
    uint32_t receiveFailures;
    uint32_t timeouts;
};

// =============================================================================
// CONVENIENCE MACROS
// =============================================================================

#define HANDLE_ERROR(handler, message) handler.handleError(message)
#define HANDLE_ERRORF(handler, format, ...) handler.handleErrorf(format, __VA_ARGS__)
#define HANDLE_TASK_ERROR(handler, message) handler.handleTaskError(message)
#define HANDLE_COMM_ERROR(handler, message) handler.handleError(message)

#endif // ERROR_HANDLER_H
