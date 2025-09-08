#ifndef ROS_LOGGER_H
#define ROS_LOGGER_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "system_config.h"
#include "mutex_wrapper.h"
#include "time_utils.h"
#include "error_handler.h"
#include "initializable.h"

// ROS2 includes
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// =============================================================================
// ROS LOGGER CLASS
// =============================================================================

class ROSLogger : public Initializable {
public:
    // Log levels
    enum LogLevel {
        LOG_DEBUG = 0,
        LOG_INFO = 1,
        LOG_WARN = 2,
        LOG_ERROR = 3,
        LOG_FATAL = 4
    };
    
    // Constructor and destructor
    ROSLogger();
    ~ROSLogger();
    
    // Initialization and cleanup
    bool initialize(rcl_node_t* node, const SystemConfig_t& config);
    void cleanup();
    
    // Logging functions
    void log(LogLevel level, const char* component, const char* message);
    void logf(LogLevel level, const char* component, const char* format, ...);
    
    // Convenience logging functions
    void debug(const char* component, const char* message);
    void info(const char* component, const char* message);
    void warn(const char* component, const char* message);
    void error(const char* component, const char* message);
    void fatal(const char* component, const char* message);
    
    // Formatted convenience functions
    void debugf(const char* component, const char* format, ...);
    void infof(const char* component, const char* format, ...);
    void warnf(const char* component, const char* format, ...);
    void errorf(const char* component, const char* format, ...);
    void fatalf(const char* component, const char* format, ...);
    
    // System status logging
    void logSystemStatus(bool systemInitialized, bool usbInitialized, 
                        bool interCoreInitialized, bool motorTaskInitialized, 
                        bool rosTaskInitialized, bool systemHealthy, 
                        uint32_t initializationErrors);
    
    void logTaskInfo(const char* taskName, uint8_t priority, uint8_t coreId, uint32_t stackSize);
    void logPerformanceMetrics(const char* taskName, uint32_t executionCount, 
                              uint32_t avgExecutionTime, uint32_t errors);
    
    // Health and error logging
    void logHealthWarning(const char* message);
    void logInitializationError(const char* component, const char* error);
    
    // Status
    bool isInitialized() const;
    bool isPublisherReady();
    
    // Public for testing
    const char* getLogLevelString(LogLevel level);
    
private:
    // ROS2 objects
    rcl_node_t* node;
    rcl_publisher_t log_publisher;
    std_msgs__msg__String log_msg;
    
    // Configuration
    SystemConfig_t config;
    bool publisherReady;
    
    // Thread safety
    MutexWrapper logMutex;
    
    // Virtual methods from Initializable
    bool doInitialize() override;
    void doCleanup() override;
    bool doHealthCheck() const override;
    
    // Private helper methods
    void publishLogMessage(const char* formattedMessage);
    void formatLogMessage(char* buffer, size_t bufferSize, LogLevel level, 
                         const char* component, const char* message);
};

// =============================================================================
// GLOBAL ROS LOGGER INSTANCE
// =============================================================================

// Global ROS logger instance
extern ROSLogger rosLogger;

// =============================================================================
// CONVENIENCE MACROS
// =============================================================================

// Convenience macros for logging
#define ROS_LOG_DEBUG(component, message) rosLogger.debug(component, message)
#define ROS_LOG_INFO(component, message) rosLogger.info(component, message)
#define ROS_LOG_WARN(component, message) rosLogger.warn(component, message)
#define ROS_LOG_ERROR(component, message) rosLogger.error(component, message)
#define ROS_LOG_FATAL(component, message) rosLogger.fatal(component, message)

#define ROS_LOG_DEBUGF(component, format, ...) rosLogger.debugf(component, format, __VA_ARGS__)
#define ROS_LOG_INFOF(component, format, ...) rosLogger.infof(component, format, __VA_ARGS__)
#define ROS_LOG_WARNF(component, format, ...) rosLogger.warnf(component, format, __VA_ARGS__)
#define ROS_LOG_ERRORF(component, format, ...) rosLogger.errorf(component, format, __VA_ARGS__)
#define ROS_LOG_FATALF(component, format, ...) rosLogger.fatalf(component, format, __VA_ARGS__)

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Initialize ROS logger
bool initializeROSLogger(rcl_node_t* node, const SystemConfig_t& config = DEFAULT_SYSTEM_CONFIG);

// Cleanup ROS logger
void cleanupROSLogger();

// Get ROS logger instance
ROSLogger* getROSLogger();

#endif // ROS_LOGGER_H
