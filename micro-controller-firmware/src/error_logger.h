#ifndef ERROR_LOGGER_H
#define ERROR_LOGGER_H

#include <stdint.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <rcl/rcl.h>
#include <std_msgs/msg/float32_multi_array.h>

enum class ErrorSeverity : uint8_t {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    CRITICAL = 4
};

enum class ErrorCategory : uint8_t {
    SYSTEM = 0,
    COMMUNICATION = 1,
    HARDWARE = 2,
    SENSOR = 3,
    MOTOR = 4,
    MEMORY = 5,
    TIMING = 6,
    RECOVERY = 7
};

struct ErrorEvent {
    uint32_t timestamp;
    ErrorSeverity severity;
    ErrorCategory category;
    uint16_t error_code;
    char description[64];
    uint32_t occurrence_count;
    uint32_t first_occurrence;
    uint32_t last_occurrence;
    bool recovery_attempted;
    bool recovery_successful;
    
    ErrorEvent() {
        timestamp = 0;
        severity = ErrorSeverity::INFO;
        category = ErrorCategory::SYSTEM;
        error_code = 0;
        description[0] = '\0';
        occurrence_count = 0;
        first_occurrence = 0;
        last_occurrence = 0;
        recovery_attempted = false;
        recovery_successful = false;
    }
};

struct ErrorPattern {
    ErrorCategory category;
    uint16_t error_code;
    uint32_t occurrence_count;
    uint32_t time_window_ms;
    float frequency_per_minute;
    bool is_recurring;
    uint32_t pattern_start_time;
    
    ErrorPattern() {
        category = ErrorCategory::SYSTEM;
        error_code = 0;
        occurrence_count = 0;
        time_window_ms = 0;
        frequency_per_minute = 0.0f;
        is_recurring = false;
        pattern_start_time = 0;
    }
};

struct RecoveryMetrics {
    uint32_t total_recovery_attempts;
    uint32_t successful_recoveries;
    uint32_t failed_recoveries;
    float success_rate;
    uint32_t average_recovery_time_ms;
    
    // Recovery attempts by category
    uint32_t communication_recoveries;
    uint32_t hardware_recoveries;
    uint32_t sensor_recoveries;
    uint32_t motor_recoveries;
    uint32_t memory_recoveries;
    
    RecoveryMetrics() {
        total_recovery_attempts = 0;
        successful_recoveries = 0;
        failed_recoveries = 0;
        success_rate = 0.0f;
        average_recovery_time_ms = 0;
        communication_recoveries = 0;
        hardware_recoveries = 0;
        sensor_recoveries = 0;
        motor_recoveries = 0;
        memory_recoveries = 0;
    }
};

class ErrorLogger {
private:
    static const size_t MAX_ERROR_EVENTS = 100;
    static const size_t MAX_ERROR_PATTERNS = 20;
    static const uint32_t PATTERN_ANALYSIS_INTERVAL_MS = 5000;  // 5 seconds
    static const uint32_t LOG_PUBLISH_INTERVAL_MS = 2000;      // 2 seconds
    static const uint32_t RECOVERY_TIMEOUT_MS = 30000;         // 30 seconds
    
    // Error storage
    ErrorEvent error_events_[MAX_ERROR_EVENTS];
    ErrorPattern error_patterns_[MAX_ERROR_PATTERNS];
    size_t event_count_;
    size_t pattern_count_;
    size_t next_event_index_;
    
    // Recovery tracking
    RecoveryMetrics recovery_metrics_;
    
    // ROS publishing
    rcl_publisher_t* error_log_publisher_;
    rcl_publisher_t* recovery_metrics_publisher_;
    std_msgs__msg__Float32MultiArray error_log_msg_;
    std_msgs__msg__Float32MultiArray recovery_msg_;
    
    // Configuration
    ErrorSeverity min_log_severity_;
    bool pattern_analysis_enabled_;
    bool recovery_tracking_enabled_;
    bool ros_publishing_enabled_;
    
    // Timing
    uint32_t last_pattern_analysis_;
    uint32_t last_log_publish_;
    
    // Thread safety
    SemaphoreHandle_t error_mutex_;
    
    // Error code definitions
    struct ErrorCodes {
        static const uint16_t SPI_COMMUNICATION_FAILURE = 1001;
        static const uint16_t ROS_CONNECTION_LOST = 2001;
        static const uint16_t ROS_MESSAGE_TIMEOUT = 2002;
        static const uint16_t ROS_PUBLISH_FAILED = 2003;
        static const uint16_t MOTOR_DRIVER_FAULT = 4001;
        static const uint16_t MOTOR_STALL_DETECTED = 4002;
        static const uint16_t MOTOR_OVERCURRENT = 4003;
        static const uint16_t SENSOR_READ_TIMEOUT = 3001;
        static const uint16_t SENSOR_DATA_INVALID = 3002;
        static const uint16_t IMU_INITIALIZATION_FAILED = 3003;
        static const uint16_t MEMORY_ALLOCATION_FAILED = 5001;
        static const uint16_t MEMORY_CORRUPTION_DETECTED = 5002;
        static const uint16_t LOOP_TIMING_VIOLATION = 6001;
        static const uint16_t WATCHDOG_TIMEOUT = 6002;
        static const uint16_t RECOVERY_ATTEMPT_STARTED = 7001;
        static const uint16_t RECOVERY_ATTEMPT_COMPLETED = 7002;
        static const uint16_t RECOVERY_ATTEMPT_FAILED = 7003;
    };
    
    // Helper methods
    void addErrorEvent(ErrorSeverity severity, ErrorCategory category, uint16_t error_code, const char* description);
    void updateErrorPattern(ErrorCategory category, uint16_t error_code);
    void analyzeErrorPatterns();
    void publishErrorLogs();
    void publishRecoveryMetrics();
    bool initializeROSMessages();
    void cleanupROSMessages();
    size_t findExistingEvent(ErrorCategory category, uint16_t error_code);
    size_t findExistingPattern(ErrorCategory category, uint16_t error_code);
    const char* severityToString(ErrorSeverity severity);
    const char* categoryToString(ErrorCategory category);

public:
    ErrorLogger();
    ~ErrorLogger();
    
    // Initialization
    bool initialize(rcl_node_t* node);
    void shutdown();
    
    // Configuration
    void setMinimumSeverity(ErrorSeverity severity);
    ErrorSeverity getMinimumSeverity() const;
    void enablePatternAnalysis(bool enable);
    void enableRecoveryTracking(bool enable);
    void enableROSPublishing(bool enable);
    
    // Error logging
    void logError(ErrorSeverity severity, ErrorCategory category, uint16_t error_code, const char* description);
    void logDebug(ErrorCategory category, const char* description);
    void logInfo(ErrorCategory category, const char* description);
    void logWarning(ErrorCategory category, const char* description);
    void logError(ErrorCategory category, const char* description);
    void logCritical(ErrorCategory category, const char* description);
    
    // Specific error logging methods
    void logSPIError(const char* description);
    void logROSConnectionError(const char* description);
    void logMotorError(const char* description);
    void logSensorError(const char* description);
    void logMemoryError(const char* description);
    void logTimingError(const char* description);
    
    // Recovery tracking
    void recordRecoveryAttempt(ErrorCategory category, const char* description);
    void recordRecoverySuccess(ErrorCategory category, uint32_t recovery_time_ms);
    void recordRecoveryFailure(ErrorCategory category, const char* reason);
    
    // Pattern analysis
    void analyzePatterns();
    bool isRecurringError(ErrorCategory category, uint16_t error_code);
    float getErrorFrequency(ErrorCategory category, uint16_t error_code);
    
    // Metrics access
    RecoveryMetrics getRecoveryMetrics() const;
    size_t getErrorCount(ErrorSeverity min_severity = ErrorSeverity::DEBUG) const;
    size_t getErrorCount(ErrorCategory category, ErrorSeverity min_severity = ErrorSeverity::DEBUG) const;
    ErrorEvent getRecentError(size_t index) const;
    ErrorPattern getErrorPattern(size_t index) const;
    
    // Main update function
    void update();
    
    // Manual publishing
    void publishCurrentLogs();
    void publishCurrentMetrics();
    
    // Error event access
    size_t getEventCount() const { return event_count_; }
    size_t getPatternCount() const { return pattern_count_; }
    
    // Clear functions
    void clearErrorHistory();
    void clearRecoveryMetrics();
    void clearErrorPatterns();
};

// Global error logger instance
extern ErrorLogger g_error_logger;

// Convenience macros for error logging
#define LOG_ERROR_DEBUG(category, desc) g_error_logger.logDebug(category, desc)
#define LOG_ERROR_INFO(category, desc) g_error_logger.logInfo(category, desc)
#define LOG_ERROR_WARNING(category, desc) g_error_logger.logWarning(category, desc)
#define LOG_ERROR_ERROR(category, desc) g_error_logger.logError(category, desc)
#define LOG_ERROR_CRITICAL(category, desc) g_error_logger.logCritical(category, desc)

// Specific error logging macros
#define LOG_SPI_ERROR(desc) g_error_logger.logSPIError(desc)
#define LOG_ROS_ERROR(desc) g_error_logger.logROSConnectionError(desc)
#define LOG_MOTOR_ERROR(desc) g_error_logger.logMotorError(desc)
#define LOG_SENSOR_ERROR(desc) g_error_logger.logSensorError(desc)
#define LOG_MEMORY_ERROR(desc) g_error_logger.logMemoryError(desc)
#define LOG_TIMING_ERROR(desc) g_error_logger.logTimingError(desc)

// Recovery tracking macros
#define RECORD_RECOVERY_ATTEMPT(category, desc) g_error_logger.recordRecoveryAttempt(category, desc)
#define RECORD_RECOVERY_SUCCESS(category, time_ms) g_error_logger.recordRecoverySuccess(category, time_ms)
#define RECORD_RECOVERY_FAILURE(category, reason) g_error_logger.recordRecoveryFailure(category, reason)

#endif // ERROR_LOGGER_H