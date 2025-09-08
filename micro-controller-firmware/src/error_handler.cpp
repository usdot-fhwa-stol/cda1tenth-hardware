#include "error_handler.h"
#include "ros_logger.h"
#include "inter_core_comm.h"
#include <stdarg.h>
#include <stdio.h>

// =============================================================================
// ERROR HANDLER IMPLEMENTATION
// =============================================================================

ErrorHandler::ErrorHandler(const char* componentName)
    : componentName(componentName), errorCount(0), lastErrorTime(0), firstErrorTime(0) {
}

void ErrorHandler::handleError(const char* errorMessage, uint8_t errorFlag) {
    errorCount++;
    lastErrorTime = xTaskGetTickCount();
    
    if (firstErrorTime == 0) {
        firstErrorTime = lastErrorTime;
    }
    
    // Call virtual method for custom handling
    onError(errorMessage, errorFlag);
    
    // Log error via ROS logger
    ROS_LOG_ERRORF(componentName, "Error: %s", errorMessage);
    
    // Report error to inter-core communication system
    if (errorFlag != 0) {
        InterCoreCommunication::getInstance().reportError(errorFlag, errorMessage);
    }
}

void ErrorHandler::handleErrorf(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    handleError(buffer);
}

bool ErrorHandler::hasRecentErrors(uint32_t timeWindowMs) const {
    if (errorCount == 0) return false;
    
    uint32_t currentTime = xTaskGetTickCount();
    return (currentTime - lastErrorTime) < pdMS_TO_TICKS(timeWindowMs);
}

bool ErrorHandler::isHealthy() const {
    // Consider unhealthy if too many recent errors
    return !hasRecentErrors(1000) && isErrorRateAcceptable(1.0f);
}

void ErrorHandler::resetErrorCount() {
    errorCount = 0;
    lastErrorTime = 0;
    firstErrorTime = 0;
}

float ErrorHandler::getErrorRate() const {
    if (errorCount == 0 || firstErrorTime == 0) return 0.0f;
    
    uint32_t currentTime = xTaskGetTickCount();
    uint32_t elapsedTicks = currentTime - firstErrorTime;
    float elapsedSeconds = (float)elapsedTicks / configTICK_RATE_HZ;
    
    return (elapsedSeconds > 0.0f) ? (float)errorCount / elapsedSeconds : 0.0f;
}

bool ErrorHandler::isErrorRateAcceptable(float maxRate) const {
    return getErrorRate() <= maxRate;
}

void ErrorHandler::onError(const char* errorMessage, uint8_t errorFlag) {
    // Default implementation does nothing
    // Subclasses can override for custom behavior
}

// =============================================================================
// TASK ERROR HANDLER IMPLEMENTATION
// =============================================================================

TaskErrorHandler::TaskErrorHandler(const char* componentName)
    : ErrorHandler(componentName), missedDeadlines(0), mutexTimeouts(0), queueOverflows(0) {
}

void TaskErrorHandler::handleTaskError(const char* errorMessage, uint8_t errorFlag) {
    handleError(errorMessage, errorFlag);
}

void TaskErrorHandler::handleDeadlineMiss() {
    missedDeadlines++;
    handleError("Task deadline missed", ERROR_FLAG_WATCHDOG_TIMEOUT);
}

void TaskErrorHandler::handleMutexTimeout() {
    mutexTimeouts++;
    handleError("Mutex timeout", ERROR_FLAG_MUTEX_TIMEOUT);
}

void TaskErrorHandler::handleQueueOverflow() {
    queueOverflows++;
    handleError("Queue overflow", ERROR_FLAG_QUEUE_OVERFLOW);
}

bool TaskErrorHandler::isTaskHealthy() const {
    if (!isHealthy()) return false;
    
    // Check for excessive missed deadlines
    if (missedDeadlines > 10) return false;
    
    return true;
}

void TaskErrorHandler::onError(const char* errorMessage, uint8_t errorFlag) {
    // Task-specific error handling can be added here
}

// =============================================================================
// COMMUNICATION ERROR HANDLER IMPLEMENTATION
// =============================================================================

CommunicationErrorHandler::CommunicationErrorHandler(const char* componentName)
    : ErrorHandler(componentName), connectionDrops(0), sendFailures(0), receiveFailures(0), timeouts(0) {
}

void CommunicationErrorHandler::handleConnectionDrop() {
    connectionDrops++;
    handleError("Connection dropped", ERROR_FLAG_COMMAND_TIMEOUT);
}

void CommunicationErrorHandler::handleMessageSendFailure() {
    sendFailures++;
    handleError("Message send failure", ERROR_FLAG_SPI_COMM_FAILURE);
}

void CommunicationErrorHandler::handleMessageReceiveFailure() {
    receiveFailures++;
    handleError("Message receive failure", ERROR_FLAG_SPI_COMM_FAILURE);
}

void CommunicationErrorHandler::handleTimeout() {
    timeouts++;
    handleError("Communication timeout", ERROR_FLAG_COMMAND_TIMEOUT);
}

bool CommunicationErrorHandler::isCommunicationHealthy() const {
    if (!isHealthy()) return false;
    
    // Check for excessive communication failures
    if (connectionDrops > 5) return false;
    if (sendFailures > 20) return false;
    if (receiveFailures > 20) return false;
    
    return true;
}

void CommunicationErrorHandler::onError(const char* errorMessage, uint8_t errorFlag) {
    // Communication-specific error handling can be added here
}
