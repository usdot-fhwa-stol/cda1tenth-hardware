#include "ros_logger.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// Global ROS logger instance
ROSLogger rosLogger;

// =============================================================================
// ROS LOGGER IMPLEMENTATION
// =============================================================================

ROSLogger::ROSLogger() 
    : Initializable("ROSLogger"),
      node(nullptr), publisherReady(false), logMutex("LogMutex") {
    
    // Initialize ROS2 objects
    log_publisher = rcl_get_zero_initialized_publisher();
    std_msgs__msg__String__init(&log_msg);
    
    config = DEFAULT_SYSTEM_CONFIG;
}

ROSLogger::~ROSLogger() {
    cleanup();
}

bool ROSLogger::initialize(rcl_node_t* node, const SystemConfig_t& config) {
    this->node = node;
    this->config = config;
    
    // Initialize log message
    log_msg.data.capacity = 512;
    log_msg.data.size = 0;
    log_msg.data.data = (char*)malloc(512);
    if (log_msg.data.data == nullptr) {
        return false;
    }
    
    // Create ROS2 publisher for logging
    rcl_ret_t ret = rclc_publisher_init_best_effort(
        &log_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "system_logs");
    
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    publisherReady = true;
    
    // Use base class initialization
    return Initializable::initialize();
}

void ROSLogger::cleanup() {
    Initializable::cleanup();
}

// Virtual method implementations from Initializable
bool ROSLogger::doInitialize() {
    return true; // Log mutex is already created in constructor
}

void ROSLogger::doCleanup() {
    if (isInitialized()) {
        rcl_publisher_fini(&log_publisher, node);
        std_msgs__msg__String__fini(&log_msg);
        
        if (log_msg.data.data != nullptr) {
            free(log_msg.data.data);
            log_msg.data.data = nullptr;
        }
    }
    
    publisherReady = false;
    node = nullptr;
}

bool ROSLogger::doHealthCheck() const {
    return publisherReady;
}

void ROSLogger::log(LogLevel level, const char* component, const char* message) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    char buffer[512];
    formatLogMessage(buffer, sizeof(buffer), level, component, message);
    publishLogMessage(buffer);
}

void ROSLogger::logf(LogLevel level, const char* component, const char* format, ...) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    char messageBuffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);
    va_end(args);
    
    log(level, component, messageBuffer);
}

void ROSLogger::debug(const char* component, const char* message) {
    log(LOG_DEBUG, component, message);
}

void ROSLogger::info(const char* component, const char* message) {
    log(LOG_INFO, component, message);
}

void ROSLogger::warn(const char* component, const char* message) {
    log(LOG_WARN, component, message);
}

void ROSLogger::error(const char* component, const char* message) {
    log(LOG_ERROR, component, message);
}

void ROSLogger::fatal(const char* component, const char* message) {
    log(LOG_FATAL, component, message);
}

void ROSLogger::debugf(const char* component, const char* format, ...) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    char messageBuffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);
    va_end(args);
    
    debug(component, messageBuffer);
}

void ROSLogger::infof(const char* component, const char* format, ...) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    char messageBuffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);
    va_end(args);
    
    info(component, messageBuffer);
}

void ROSLogger::warnf(const char* component, const char* format, ...) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    char messageBuffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);
    va_end(args);
    
    warn(component, messageBuffer);
}

void ROSLogger::errorf(const char* component, const char* format, ...) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    char messageBuffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);
    va_end(args);
    
    error(component, messageBuffer);
}

void ROSLogger::fatalf(const char* component, const char* format, ...) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    char messageBuffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);
    va_end(args);
    
    fatal(component, messageBuffer);
}

void ROSLogger::logSystemStatus(bool systemInitialized, bool usbInitialized, 
                               bool interCoreInitialized, bool motorTaskInitialized, 
                               bool rosTaskInitialized, bool systemHealthy, 
                               uint32_t initializationErrors) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    infof("SYSTEM", "System Initialized: %s", systemInitialized ? "YES" : "NO");
    infof("SYSTEM", "USB CDC: %s", usbInitialized ? "OK" : "FAIL");
    infof("SYSTEM", "Inter-Core Comm: %s", interCoreInitialized ? "OK" : "FAIL");
    infof("SYSTEM", "Motor Control Task: %s", motorTaskInitialized ? "OK" : "FAIL");
    infof("SYSTEM", "ROS Comm Task: %s", rosTaskInitialized ? "OK" : "FAIL");
    infof("SYSTEM", "System Health: %s", systemHealthy ? "HEALTHY" : "UNHEALTHY");
    
    if (initializationErrors > 0) {
        errorf("SYSTEM", "Initialization Errors: 0x%08X", initializationErrors);
    }
}

void ROSLogger::logTaskInfo(const char* taskName, uint8_t priority, uint8_t coreId, uint32_t stackSize) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    infof("TASK", "Task Created: %s | Core: %d | Priority: %d | Stack: %d bytes",
          taskName, coreId, priority, stackSize);
}

void ROSLogger::logPerformanceMetrics(const char* taskName, uint32_t executionCount, 
                                     uint32_t avgExecutionTime, uint32_t errors) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    infof("PERF", "%s Performance - Executions: %u, Avg Time: %u us, Errors: %u",
          taskName, executionCount, avgExecutionTime, errors);
}

void ROSLogger::logHealthWarning(const char* message) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    warn("HEALTH", message);
}

void ROSLogger::logInitializationError(const char* component, const char* error) {
    if (!isInitialized() || !publisherReady) {
        return;
    }
    
    errorf("INIT", "INIT ERROR [%s]: %s", component, error);
}

bool ROSLogger::isInitialized() const {
    return Initializable::isInitialized();
}

bool ROSLogger::isPublisherReady() {
    return publisherReady;
}

const char* ROSLogger::getLogLevelString(LogLevel level) {
    switch (level) {
        case LOG_DEBUG: return "DEBUG";
        case LOG_INFO:  return "INFO";
        case LOG_WARN:  return "WARN";
        case LOG_ERROR: return "ERROR";
        case LOG_FATAL: return "FATAL";
        default:        return "UNKNOWN";
    }
}

void ROSLogger::publishLogMessage(const char* formattedMessage) {
    // Use RAII mutex lock
    MutexWrapper::LockGuard lock(logMutex);
    if (!lock.isLocked()) {
        errorHandler.handleError("Failed to acquire log mutex");
        return;
    }
    
    // Copy message to ROS message
    size_t messageLen = strlen(formattedMessage);
    if (messageLen >= log_msg.data.capacity) {
        messageLen = log_msg.data.capacity - 1;
    }
    
    strncpy(log_msg.data.data, formattedMessage, messageLen);
    log_msg.data.data[messageLen] = '\0';
    log_msg.data.size = messageLen;
    
    // Publish message
    rcl_ret_t ret = rcl_publish(&log_publisher, &log_msg, NULL);
    if (ret != RCL_RET_OK) {
        errorHandler.handleError("Failed to publish log message");
    }
}

void ROSLogger::formatLogMessage(char* buffer, size_t bufferSize, LogLevel level, 
                                const char* component, const char* message) {
    uint32_t timestamp = TimeUtils::getCurrentTimestampMs();
    const char* levelStr = getLogLevelString(level);
    
    snprintf(buffer, bufferSize, "[%u] [%s] [%s]: %s", 
             timestamp, levelStr, component, message);
}


// =============================================================================
// GLOBAL UTILITY FUNCTIONS
// =============================================================================

bool initializeROSLogger(rcl_node_t* node, const SystemConfig_t& config) {
    return rosLogger.initialize(node, config);
}

void cleanupROSLogger() {
    rosLogger.cleanup();
}

ROSLogger* getROSLogger() {
    return &rosLogger;
}
