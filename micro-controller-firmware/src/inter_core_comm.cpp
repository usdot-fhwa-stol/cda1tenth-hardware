#include "inter_core_comm.h"
#include <string.h>

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

bool copyMotorCommand(const MotorCommand_t& src, MotorCommand_t& dst) {
    if (!MotorValidation::validateMotorCommand(src)) {
        return false;
    }
    memcpy(&dst, &src, sizeof(MotorCommand_t));
    return true;
}

bool copyMotorStatus(const MotorStatus_t& src, MotorStatus_t& dst) {
    if (!MotorValidation::validateMotorStatus(src)) {
        return false;
    }
    memcpy(&dst, &src, sizeof(MotorStatus_t));
    return true;
}

uint8_t calculateQueueUtilization(QueueHandle_t queue) {
    if (queue == NULL) return 0;
    
    // Get the number of messages currently in the queue
    UBaseType_t queueWaiting = uxQueueMessagesWaiting(queue);
    
    // For utilization calculation, we'll use a fixed maximum queue size
    // since uxQueueGetQueueLength is not available in this FreeRTOS version
    const UBaseType_t MAX_QUEUE_SIZE = MAX_QUEUE_SIZE_FOR_UTILIZATION; // Use configuration constant
    
    return (uint8_t)((queueWaiting * 100) / MAX_QUEUE_SIZE);
}

// =============================================================================
// THREAD-SAFE CAR STATE IMPLEMENTATION
// =============================================================================

ThreadSafeCarState::ThreadSafeCarState() 
    : Initializable("ThreadSafeCarState"),
      commandMutex("Command"), statusMutex("Status"), errorCounterMutex("ErrorCounter"),
      commandQueue(NULL), statusQueue(NULL),
      newCommandAvailable(false),
      commandQueueOverruns(0), statusQueueOverruns(0), mutexTimeouts(0) {
    
    // Initialize structures with default values
    motor_command_init(&currentCommand);
    motor_status_init(&currentStatus);
    config = DEFAULT_SYSTEM_CONFIG;
}

ThreadSafeCarState::~ThreadSafeCarState() {
    // Clean up queues
    if (commandQueue != NULL) {
        vQueueDelete(commandQueue);
    }
    if (statusQueue != NULL) {
        vQueueDelete(statusQueue);
    }
}

bool ThreadSafeCarState::initialize() {
    return Initializable::initialize();
}

// Virtual method implementations from Initializable
bool ThreadSafeCarState::doInitialize() {
    // Create queues
    commandQueue = xQueueCreate(config.command_queue_size, sizeof(MotorCommand_t));
    statusQueue = xQueueCreate(config.status_queue_size, sizeof(MotorStatus_t));
    
    if (commandQueue == NULL || statusQueue == NULL) {
        return false;
    }
    
    return true;
}

void ThreadSafeCarState::doCleanup() {
    // Clean up queues
    if (commandQueue != NULL) {
        vQueueDelete(commandQueue);
        commandQueue = NULL;
    }
    if (statusQueue != NULL) {
        vQueueDelete(statusQueue);
        statusQueue = NULL;
    }
}

bool ThreadSafeCarState::doHealthCheck() const {
    // Check if queues are valid
    if (commandQueue == NULL || statusQueue == NULL) {
        return false;
    }
    
    // Check error rates (consider unhealthy if too many errors)
    if (commandQueueOverruns > MAX_ACCEPTABLE_ERRORS ||
        statusQueueOverruns > MAX_ACCEPTABLE_ERRORS ||
        mutexTimeouts > MAX_ACCEPTABLE_ERRORS) {
        return false;
    }
    
    return true;
}

bool ThreadSafeCarState::setCommand(const MotorCommand_t& cmd) {
    if (!isInitialized() || !MotorValidation::validateMotorCommand(cmd)) {
        return false;
    }
    
    if (commandMutex.take()) {
        copyMotorCommand(cmd, currentCommand);
        newCommandAvailable = true;
        commandMutex.give();
        return true;
    }
    
    mutexTimeouts++;
    return false;
}

bool ThreadSafeCarState::getCommand(MotorCommand_t& cmd) {
    if (!isInitialized()) {
        return false;
    }
    
    if (commandMutex.take()) {
        copyMotorCommand(currentCommand, cmd);
        commandMutex.give();
        return true;
    }
    
    mutexTimeouts++;
    return false;
}

bool ThreadSafeCarState::hasNewCommand() {
    return newCommandAvailable;
}

void ThreadSafeCarState::clearNewCommandFlag() {
    if (commandMutex.take()) {
        newCommandAvailable = false;
        commandMutex.give();
    }
}

bool ThreadSafeCarState::setStatus(const MotorStatus_t& status) {
    if (!isInitialized() || !MotorValidation::validateMotorStatus(status)) {
        return false;
    }
    
    if (statusMutex.take()) {
        copyMotorStatus(status, currentStatus);
        statusMutex.give();
        return true;
    }
    
    mutexTimeouts++;
    return false;
}

bool ThreadSafeCarState::getStatus(MotorStatus_t& status) {
    if (!isInitialized()) {
        return false;
    }
    
    if (statusMutex.take()) {
        copyMotorStatus(currentStatus, status);
        statusMutex.give();
        return true;
    }
    
    mutexTimeouts++;
    return false;
}

bool ThreadSafeCarState::sendCommandToQueue(const MotorCommand_t& cmd) {
    if (!isInitialized() || commandQueue == NULL || !MotorValidation::validateMotorCommand(cmd)) {
        return false;
    }
    
    BaseType_t result = xQueueSend(commandQueue, &cmd, 0);  // Non-blocking
    
    if (result != pdTRUE) {
        // Queue is full, try to handle overflow
        if (handleCommandQueueOverflow(cmd)) {
            commandQueueOverruns++;
            return true;  // Overflow handled successfully
        }
        commandQueueOverruns++;
        return false;
    }
    
    return true;
}

bool ThreadSafeCarState::receiveCommandFromQueue(MotorCommand_t& cmd, TickType_t timeout) {
    if (!isInitialized() || commandQueue == NULL) {
        return false;
    }
    
    BaseType_t result = xQueueReceive(commandQueue, &cmd, timeout);
    return (result == pdTRUE) && MotorValidation::validateMotorCommand(cmd);
}

bool ThreadSafeCarState::sendStatusToQueue(const MotorStatus_t& status) {
    if (!isInitialized() || statusQueue == NULL || !MotorValidation::validateMotorStatus(status)) {
        return false;
    }
    
    BaseType_t result = xQueueSend(statusQueue, &status, 0);  // Non-blocking
    
    if (result != pdTRUE) {
        // Queue is full, try to handle overflow
        if (handleStatusQueueOverflow(status)) {
            statusQueueOverruns++;
            return true;  // Overflow handled successfully
        }
        statusQueueOverruns++;
        return false;
    }
    
    return true;
}

bool ThreadSafeCarState::receiveStatusFromQueue(MotorStatus_t& status, TickType_t timeout) {
    if (!isInitialized() || statusQueue == NULL) {
        return false;
    }
    
    BaseType_t result = xQueueReceive(statusQueue, &status, timeout);
    return (result == pdTRUE) && MotorValidation::validateMotorStatus(status);
}

uint32_t ThreadSafeCarState::getCommandQueueWaiting() {
    if (commandQueue == NULL) return 0;
    return uxQueueMessagesWaiting(commandQueue);
}

uint32_t ThreadSafeCarState::getStatusQueueWaiting() {
    if (statusQueue == NULL) return 0;
    return uxQueueMessagesWaiting(statusQueue);
}

uint32_t ThreadSafeCarState::getCommandQueueSpacesAvailable() {
    if (commandQueue == NULL) return 0;
    return uxQueueSpacesAvailable(commandQueue);
}

uint32_t ThreadSafeCarState::getStatusQueueSpacesAvailable() {
    if (statusQueue == NULL) return 0;
    return uxQueueSpacesAvailable(statusQueue);
}

uint32_t ThreadSafeCarState::getCommandQueueOverruns() {
    return commandQueueOverruns;
}

uint32_t ThreadSafeCarState::getStatusQueueOverruns() {
    return statusQueueOverruns;
}

uint32_t ThreadSafeCarState::getMutexTimeouts() {
    return mutexTimeouts;
}

void ThreadSafeCarState::resetErrorCounters() {
    if (errorCounterMutex.take()) {
        commandQueueOverruns = 0;
        statusQueueOverruns = 0;
        mutexTimeouts = 0;
        errorCounterMutex.give();
    }
}

bool ThreadSafeCarState::isHealthy() const {
    return Initializable::isHealthy();
}

// Private helper methods

bool ThreadSafeCarState::handleCommandQueueOverflow(const MotorCommand_t& cmd) {
    // Strategy: Remove oldest command and add new one
    MotorCommand_t discardedCmd;
    
    if (xQueueReceive(commandQueue, &discardedCmd, 0) == pdTRUE) {
        // Successfully removed oldest command, now try to add new one
        return xQueueSend(commandQueue, &cmd, 0) == pdTRUE;
    }
    
    return false;  // Could not handle overflow
}

bool ThreadSafeCarState::handleStatusQueueOverflow(const MotorStatus_t& status) {
    // Strategy: Remove oldest status and add new one
    MotorStatus_t discardedStatus;
    
    if (xQueueReceive(statusQueue, &discardedStatus, 0) == pdTRUE) {
        // Successfully removed oldest status, now try to add new one
        return xQueueSend(statusQueue, &status, 0) == pdTRUE;
    }
    
    return false;  // Could not handle overflow
}

// Enhanced queue operations with ISR support
bool ThreadSafeCarState::sendCommandToQueueFromISR(const MotorCommand_t& cmd, BaseType_t* pxHigherPriorityTaskWoken) {
    if (!isInitialized() || commandQueue == NULL || !MotorValidation::validateMotorCommand(cmd)) {
        return false;
    }
    
    BaseType_t result = xQueueSendFromISR(commandQueue, &cmd, pxHigherPriorityTaskWoken);
    
    if (result != pdTRUE) {
        // Queue is full, handle overflow in ISR context
        MotorCommand_t discardedCmd;
        if (xQueueReceiveFromISR(commandQueue, &discardedCmd, pxHigherPriorityTaskWoken) == pdTRUE) {
            result = xQueueSendFromISR(commandQueue, &cmd, pxHigherPriorityTaskWoken);
        }
        if (result != pdTRUE) {
            commandQueueOverruns++;
        }
    }
    
    return result == pdTRUE;
}

bool ThreadSafeCarState::sendStatusToQueueFromISR(const MotorStatus_t& status, BaseType_t* pxHigherPriorityTaskWoken) {
    if (!isInitialized() || statusQueue == NULL || !MotorValidation::validateMotorStatus(status)) {
        return false;
    }
    
    BaseType_t result = xQueueSendFromISR(statusQueue, &status, pxHigherPriorityTaskWoken);
    
    if (result != pdTRUE) {
        // Queue is full, handle overflow in ISR context
        MotorStatus_t discardedStatus;
        if (xQueueReceiveFromISR(statusQueue, &discardedStatus, pxHigherPriorityTaskWoken) == pdTRUE) {
            result = xQueueSendFromISR(statusQueue, &status, pxHigherPriorityTaskWoken);
        }
        if (result != pdTRUE) {
            statusQueueOverruns++;
        }
    }
    
    return result == pdTRUE;
}

bool ThreadSafeCarState::receiveCommandFromQueueFromISR(MotorCommand_t& cmd, BaseType_t* pxHigherPriorityTaskWoken) {
    if (!isInitialized() || commandQueue == NULL) {
        return false;
    }
    
    BaseType_t result = xQueueReceiveFromISR(commandQueue, &cmd, pxHigherPriorityTaskWoken);
    return (result == pdTRUE) && MotorValidation::validateMotorCommand(cmd);
}

bool ThreadSafeCarState::receiveStatusFromQueueFromISR(MotorStatus_t& status, BaseType_t* pxHigherPriorityTaskWoken) {
    if (!isInitialized() || statusQueue == NULL) {
        return false;
    }
    
    BaseType_t result = xQueueReceiveFromISR(statusQueue, &status, pxHigherPriorityTaskWoken);
    return (result == pdTRUE) && MotorValidation::validateMotorStatus(status);
}

// Queue management methods
void ThreadSafeCarState::clearCommandQueue() {
    if (commandQueue != NULL) {
        xQueueReset(commandQueue);
    }
}

void ThreadSafeCarState::clearStatusQueue() {
    if (statusQueue != NULL) {
        xQueueReset(statusQueue);
    }
}

bool ThreadSafeCarState::isCommandQueueFull() {
    if (commandQueue == NULL) return true;
    return uxQueueSpacesAvailable(commandQueue) == 0;
}

bool ThreadSafeCarState::isStatusQueueFull() {
    if (statusQueue == NULL) return true;
    return uxQueueSpacesAvailable(statusQueue) == 0;
}

// Detailed statistics methods
ThreadSafeCarState::QueueStatistics ThreadSafeCarState::getCommandQueueStatistics() {
    QueueStatistics stats = {0};
    
    if (commandQueue != NULL) {
        stats.waiting_messages = uxQueueMessagesWaiting(commandQueue);
        stats.available_spaces = uxQueueSpacesAvailable(commandQueue);
        stats.overruns = commandQueueOverruns;
        stats.is_full = (stats.available_spaces == 0);
        
        // Calculate utilization percentage
        const UBaseType_t total_capacity = stats.waiting_messages + stats.available_spaces;
        if (total_capacity > 0) {
            stats.utilization_percent = (stats.waiting_messages * 100) / total_capacity;
        }
    }
    
    return stats;
}

ThreadSafeCarState::QueueStatistics ThreadSafeCarState::getStatusQueueStatistics() {
    QueueStatistics stats = {0};
    
    if (statusQueue != NULL) {
        stats.waiting_messages = uxQueueMessagesWaiting(statusQueue);
        stats.available_spaces = uxQueueSpacesAvailable(statusQueue);
        stats.overruns = statusQueueOverruns;
        stats.is_full = (stats.available_spaces == 0);
        
        // Calculate utilization percentage
        const UBaseType_t total_capacity = stats.waiting_messages + stats.available_spaces;
        if (total_capacity > 0) {
            stats.utilization_percent = (stats.waiting_messages * 100) / total_capacity;
        }
    }
    
    return stats;
}

// =============================================================================
// INTER-CORE COMMUNICATION MANAGER IMPLEMENTATION
// =============================================================================

InterCoreCommunication::InterCoreCommunication() 
    : Initializable("InterCoreCommunication"),
      errorFlags(0), errorMutex("ErrorMutex") {
    systemConfig = DEFAULT_SYSTEM_CONFIG;
}

InterCoreCommunication::~InterCoreCommunication() {
    cleanup();
}

InterCoreCommunication& InterCoreCommunication::getInstance() {
    static InterCoreCommunication instance;
    return instance;
}

bool InterCoreCommunication::initialize(const SystemConfig_t& config) {
    systemConfig = config;
    
    // Initialize car state
    if (!carState.initialize()) {
        return false;
    }
    
    // Use base class initialization
    return Initializable::initialize();
}

void InterCoreCommunication::cleanup() {
    Initializable::cleanup();
}

// Virtual method implementations from Initializable
bool InterCoreCommunication::doInitialize() {
    return true; // Error mutex is already created in constructor
}

void InterCoreCommunication::doCleanup() {
    errorFlags = 0;
}

bool InterCoreCommunication::doHealthCheck() const {
    return carState.isHealthy();
}

ThreadSafeCarState& InterCoreCommunication::getCarState() {
    return carState;
}

void InterCoreCommunication::updatePerformanceMetrics(PerformanceMetrics_t& metrics) {
    if (!isInitialized()) return;
    
    // Update queue statistics
    metrics.command_queue_overruns = carState.getCommandQueueOverruns();
    metrics.status_queue_overruns = carState.getStatusQueueOverruns();
    metrics.mutex_timeout_count = carState.getMutexTimeouts();
    
    // Update queue depth statistics
    metrics.command_queue_max_depth = max(metrics.command_queue_max_depth, 
                                         carState.getCommandQueueWaiting());
    metrics.status_queue_max_depth = max(metrics.status_queue_max_depth, 
                                        carState.getStatusQueueWaiting());
    
    // Update timestamp
    metrics.last_update_timestamp = TimeUtils::getCurrentTimestampUs();
}

bool InterCoreCommunication::isHealthy() const {
    if (!Initializable::isHealthy()) return false;
    
    // Check error flags (no critical errors should be present)
    const uint8_t CRITICAL_ERROR_MASK = ERROR_FLAG_MEMORY_ALLOCATION | 
                                       ERROR_FLAG_WATCHDOG_TIMEOUT;
    
    if (errorFlags & CRITICAL_ERROR_MASK) return false;
    
    return true;
}

void InterCoreCommunication::reportError(uint8_t errorFlag, const char* description) {
    if (errorMutex.take(pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
        errorFlags |= errorFlag;
        errorMutex.give();
        
        // Optional: Log error description if provided
        if (description != nullptr) {
            // Note: Can't use ROS logger here as it may not be initialized yet
            // This will be handled by the calling system
        }
    }
}

uint8_t InterCoreCommunication::getErrorFlags() {
    uint8_t flags = 0;
    
    if (errorMutex.take(pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
        flags = errorFlags;
        errorMutex.give();
    }
    
    return flags;
}

void InterCoreCommunication::clearErrorFlags() {
    if (errorMutex.take(pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
        errorFlags = 0;
        errorMutex.give();
    }
}