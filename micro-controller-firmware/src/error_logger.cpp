#include "error_logger.h"
#include <rclc/rclc.h>
#include <string.h>
#include <Arduino.h>
#include <algorithm>

// Global instance
ErrorLogger g_error_logger;

ErrorLogger::ErrorLogger() 
    : event_count_(0)
    , pattern_count_(0)
    , next_event_index_(0)
    , error_log_publisher_(nullptr)
    , recovery_metrics_publisher_(nullptr)
    , min_log_severity_(ErrorSeverity::INFO)
    , pattern_analysis_enabled_(true)
    , recovery_tracking_enabled_(true)
    , ros_publishing_enabled_(true)
    , last_pattern_analysis_(0)
    , last_log_publish_(0)
{
    error_mutex_ = xSemaphoreCreateMutex();
    
    // Initialize error events array
    for (size_t i = 0; i < MAX_ERROR_EVENTS; i++) {
        error_events_[i] = ErrorEvent();
    }
    
    // Initialize error patterns array
    for (size_t i = 0; i < MAX_ERROR_PATTERNS; i++) {
        error_patterns_[i] = ErrorPattern();
    }
}

ErrorLogger::~ErrorLogger() {
    shutdown();
    if (error_mutex_ != NULL) {
        vSemaphoreDelete(error_mutex_);
    }
}

bool ErrorLogger::initialize(rcl_node_t* node) {
    if (node == NULL) return false;
    
    // Initialize error log publisher
    error_log_publisher_ = (rcl_publisher_t*)malloc(sizeof(rcl_publisher_t));
    if (error_log_publisher_ == NULL) return false;
    
    rcl_ret_t ret = rclc_publisher_init_best_effort(
        error_log_publisher_, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "error_log"
    );
    
    if (ret != RCL_RET_OK) {
        free(error_log_publisher_);
        error_log_publisher_ = nullptr;
        return false;
    }
    
    // Initialize recovery metrics publisher
    recovery_metrics_publisher_ = (rcl_publisher_t*)malloc(sizeof(rcl_publisher_t));
    if (recovery_metrics_publisher_ == NULL) {
        rcl_publisher_fini(error_log_publisher_, node);
        free(error_log_publisher_);
        error_log_publisher_ = nullptr;
        return false;
    }
    
    ret = rclc_publisher_init_best_effort(
        recovery_metrics_publisher_, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "recovery_metrics"
    );
    
    if (ret != RCL_RET_OK) {
        rcl_publisher_fini(error_log_publisher_, node);
        free(error_log_publisher_);
        free(recovery_metrics_publisher_);
        error_log_publisher_ = nullptr;
        recovery_metrics_publisher_ = nullptr;
        return false;
    }
    
    // Initialize ROS messages
    if (!initializeROSMessages()) {
        shutdown();
        return false;
    }
    
    return true;
}

void ErrorLogger::shutdown() {
    cleanupROSMessages();
    
    if (error_log_publisher_ != nullptr) {
        free(error_log_publisher_);
        error_log_publisher_ = nullptr;
    }
    
    if (recovery_metrics_publisher_ != nullptr) {
        free(recovery_metrics_publisher_);
        recovery_metrics_publisher_ = nullptr;
    }
}

bool ErrorLogger::initializeROSMessages() {
    // Initialize error log message
    error_log_msg_.data.size = 0;
    error_log_msg_.data.capacity = 10;  // Severity, category, code, timestamp, count, etc.
    error_log_msg_.data.data = (float*)malloc(10 * sizeof(float));
    if (error_log_msg_.data.data == NULL) return false;
    
    // Initialize recovery metrics message
    recovery_msg_.data.size = 0;
    recovery_msg_.data.capacity = 10;  // Various recovery metrics
    recovery_msg_.data.data = (float*)malloc(10 * sizeof(float));
    if (recovery_msg_.data.data == NULL) {
        free(error_log_msg_.data.data);
        error_log_msg_.data.data = NULL;
        return false;
    }
    
    return true;
}

void ErrorLogger::cleanupROSMessages() {
    if (error_log_msg_.data.data != NULL) {
        free(error_log_msg_.data.data);
        error_log_msg_.data.data = NULL;
    }
    
    if (recovery_msg_.data.data != NULL) {
        free(recovery_msg_.data.data);
        recovery_msg_.data.data = NULL;
    }
}

void ErrorLogger::setMinimumSeverity(ErrorSeverity severity) {
    min_log_severity_ = severity;
}

ErrorSeverity ErrorLogger::getMinimumSeverity() const {
    return min_log_severity_;
}

void ErrorLogger::enablePatternAnalysis(bool enable) {
    pattern_analysis_enabled_ = enable;
}

void ErrorLogger::enableRecoveryTracking(bool enable) {
    recovery_tracking_enabled_ = enable;
}

void ErrorLogger::enableROSPublishing(bool enable) {
    ros_publishing_enabled_ = enable;
}

void ErrorLogger::logError(ErrorSeverity severity, ErrorCategory category, uint16_t error_code, const char* description) {
    if (severity < min_log_severity_) return;
    
    if (error_mutex_ != NULL && xSemaphoreTake(error_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        addErrorEvent(severity, category, error_code, description);
        
        if (pattern_analysis_enabled_) {
            updateErrorPattern(category, error_code);
        }
        
        xSemaphoreGive(error_mutex_);
    }
}

void ErrorLogger::logDebug(ErrorCategory category, const char* description) {
    logError(ErrorSeverity::DEBUG, category, 0, description);
}

void ErrorLogger::logInfo(ErrorCategory category, const char* description) {
    logError(ErrorSeverity::INFO, category, 0, description);
}

void ErrorLogger::logWarning(ErrorCategory category, const char* description) {
    logError(ErrorSeverity::WARNING, category, 0, description);
}

void ErrorLogger::logError(ErrorCategory category, const char* description) {
    logError(ErrorSeverity::ERROR, category, 0, description);
}

void ErrorLogger::logCritical(ErrorCategory category, const char* description) {
    logError(ErrorSeverity::CRITICAL, category, 0, description);
}

void ErrorLogger::logSPIError(const char* description) {
    logError(ErrorSeverity::ERROR, ErrorCategory::HARDWARE, ErrorCodes::SPI_COMMUNICATION_FAILURE, description);
}

void ErrorLogger::logROSConnectionError(const char* description) {
    logError(ErrorSeverity::WARNING, ErrorCategory::COMMUNICATION, ErrorCodes::ROS_CONNECTION_LOST, description);
}

void ErrorLogger::logMotorError(const char* description) {
    logError(ErrorSeverity::ERROR, ErrorCategory::MOTOR, ErrorCodes::MOTOR_DRIVER_FAULT, description);
}

void ErrorLogger::logSensorError(const char* description) {
    logError(ErrorSeverity::WARNING, ErrorCategory::SENSOR, ErrorCodes::SENSOR_READ_TIMEOUT, description);
}

void ErrorLogger::logMemoryError(const char* description) {
    logError(ErrorSeverity::CRITICAL, ErrorCategory::MEMORY, ErrorCodes::MEMORY_ALLOCATION_FAILED, description);
}

void ErrorLogger::logTimingError(const char* description) {
    logError(ErrorSeverity::WARNING, ErrorCategory::TIMING, ErrorCodes::LOOP_TIMING_VIOLATION, description);
}

void ErrorLogger::recordRecoveryAttempt(ErrorCategory category, const char* description) {
    if (!recovery_tracking_enabled_) return;
    
    if (error_mutex_ != NULL && xSemaphoreTake(error_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        recovery_metrics_.total_recovery_attempts++;
        
        // Increment category-specific counter
        switch (category) {
            case ErrorCategory::COMMUNICATION:
                recovery_metrics_.communication_recoveries++;
                break;
            case ErrorCategory::HARDWARE:
                recovery_metrics_.hardware_recoveries++;
                break;
            case ErrorCategory::SENSOR:
                recovery_metrics_.sensor_recoveries++;
                break;
            case ErrorCategory::MOTOR:
                recovery_metrics_.motor_recoveries++;
                break;
            case ErrorCategory::MEMORY:
                recovery_metrics_.memory_recoveries++;
                break;
            default:
                break;
        }
        
        // Log the recovery attempt
        logError(ErrorSeverity::INFO, ErrorCategory::RECOVERY, ErrorCodes::RECOVERY_ATTEMPT_STARTED, description);
        
        xSemaphoreGive(error_mutex_);
    }
}

void ErrorLogger::recordRecoverySuccess(ErrorCategory category, uint32_t recovery_time_ms) {
    if (!recovery_tracking_enabled_) return;
    
    if (error_mutex_ != NULL && xSemaphoreTake(error_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        recovery_metrics_.successful_recoveries++;
        
        // Update average recovery time
        uint32_t total_recoveries = recovery_metrics_.successful_recoveries + recovery_metrics_.failed_recoveries;
        if (total_recoveries > 0) {
            recovery_metrics_.average_recovery_time_ms = 
                (recovery_metrics_.average_recovery_time_ms * (total_recoveries - 1) + recovery_time_ms) / total_recoveries;
        }
        
        // Update success rate
        recovery_metrics_.success_rate = 
            (float)recovery_metrics_.successful_recoveries / (float)recovery_metrics_.total_recovery_attempts;
        
        // Log the successful recovery
        logError(ErrorSeverity::INFO, ErrorCategory::RECOVERY, ErrorCodes::RECOVERY_ATTEMPT_COMPLETED, "recovery_success");
        
        xSemaphoreGive(error_mutex_);
    }
}

void ErrorLogger::recordRecoveryFailure(ErrorCategory category, const char* reason) {
    if (!recovery_tracking_enabled_) return;
    
    if (error_mutex_ != NULL && xSemaphoreTake(error_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        recovery_metrics_.failed_recoveries++;
        
        // Update success rate
        recovery_metrics_.success_rate = 
            (float)recovery_metrics_.successful_recoveries / (float)recovery_metrics_.total_recovery_attempts;
        
        // Log the failed recovery
        logError(ErrorSeverity::ERROR, ErrorCategory::RECOVERY, ErrorCodes::RECOVERY_ATTEMPT_FAILED, reason);
        
        xSemaphoreGive(error_mutex_);
    }
}

void ErrorLogger::addErrorEvent(ErrorSeverity severity, ErrorCategory category, uint16_t error_code, const char* description) {
    uint32_t now = millis();
    
    // Check if this error already exists
    size_t existing_index = findExistingEvent(category, error_code);
    
    if (existing_index < MAX_ERROR_EVENTS) {
        // Update existing event
        ErrorEvent& event = error_events_[existing_index];
        event.occurrence_count++;
        event.last_occurrence = now;
        event.timestamp = now;  // Update to latest occurrence
    } else {
        // Create new event
        ErrorEvent& event = error_events_[next_event_index_];
        event.timestamp = now;
        event.severity = severity;
        event.category = category;
        event.error_code = error_code;
        strncpy(event.description, description, sizeof(event.description) - 1);
        event.description[sizeof(event.description) - 1] = '\0';
        event.occurrence_count = 1;
        event.first_occurrence = now;
        event.last_occurrence = now;
        event.recovery_attempted = false;
        event.recovery_successful = false;
        
        // Update indices
        next_event_index_ = (next_event_index_ + 1) % MAX_ERROR_EVENTS;
        if (event_count_ < MAX_ERROR_EVENTS) {
            event_count_++;
        }
    }
}

void ErrorLogger::updateErrorPattern(ErrorCategory category, uint16_t error_code) {
    uint32_t now = millis();
    
    // Find existing pattern or create new one
    size_t pattern_index = findExistingPattern(category, error_code);
    
    if (pattern_index >= MAX_ERROR_PATTERNS) {
        // Create new pattern if we have space
        if (pattern_count_ < MAX_ERROR_PATTERNS) {
            pattern_index = pattern_count_;
            pattern_count_++;
            
            ErrorPattern& pattern = error_patterns_[pattern_index];
            pattern.category = category;
            pattern.error_code = error_code;
            pattern.occurrence_count = 0;
            pattern.pattern_start_time = now;
        } else {
            return;  // No space for new patterns
        }
    }
    
    // Update pattern
    ErrorPattern& pattern = error_patterns_[pattern_index];
    pattern.occurrence_count++;
    
    // Calculate frequency
    uint32_t time_window = now - pattern.pattern_start_time;
    if (time_window > 0) {
        pattern.frequency_per_minute = (pattern.occurrence_count * 60000.0f) / time_window;
        pattern.time_window_ms = time_window;
        
        // Determine if it's recurring (more than 3 occurrences in 5 minutes)
        pattern.is_recurring = (pattern.occurrence_count >= 3) && (time_window <= 300000);
    }
}

void ErrorLogger::analyzeErrorPatterns() {
    if (!pattern_analysis_enabled_) return;
    
    uint32_t now = millis();
    
    for (size_t i = 0; i < pattern_count_; i++) {
        ErrorPattern& pattern = error_patterns_[i];
        
        // Update frequency calculations
        uint32_t time_window = now - pattern.pattern_start_time;
        if (time_window > 0) {
            pattern.frequency_per_minute = (pattern.occurrence_count * 60000.0f) / time_window;
            pattern.time_window_ms = time_window;
            
            // Check for recurring patterns
            if (pattern.occurrence_count >= 5 && pattern.frequency_per_minute > 1.0f) {
                if (!pattern.is_recurring) {
                    pattern.is_recurring = true;
                    // Log recurring pattern detection
                    logError(ErrorSeverity::WARNING, ErrorCategory::SYSTEM, 9999, "recurring_error_pattern");
                }
            }
        }
    }
}

void ErrorLogger::publishErrorLogs() {
    if (!ros_publishing_enabled_ || error_log_publisher_ == NULL) return;
    
    if (error_mutex_ != NULL && xSemaphoreTake(error_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        
        if (error_log_msg_.data.data != NULL && event_count_ > 0) {
            // Publish most recent error
            size_t recent_index = (next_event_index_ + MAX_ERROR_EVENTS - 1) % MAX_ERROR_EVENTS;
            const ErrorEvent& recent_error = error_events_[recent_index];
            
            error_log_msg_.data.size = 8;
            error_log_msg_.data.data[0] = (float)recent_error.severity;
            error_log_msg_.data.data[1] = (float)recent_error.category;
            error_log_msg_.data.data[2] = (float)recent_error.error_code;
            error_log_msg_.data.data[3] = (float)recent_error.timestamp;
            error_log_msg_.data.data[4] = (float)recent_error.occurrence_count;
            error_log_msg_.data.data[5] = (float)event_count_;
            error_log_msg_.data.data[6] = (float)pattern_count_;
            error_log_msg_.data.data[7] = recent_error.recovery_successful ? 1.0f : 0.0f;
            
            rcl_publish(error_log_publisher_, &error_log_msg_, NULL);
        }
        
        xSemaphoreGive(error_mutex_);
    }
}

void ErrorLogger::publishRecoveryMetrics() {
    if (!ros_publishing_enabled_ || recovery_metrics_publisher_ == NULL) return;
    
    if (error_mutex_ != NULL && xSemaphoreTake(error_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        
        if (recovery_msg_.data.data != NULL) {
            recovery_msg_.data.size = 10;
            recovery_msg_.data.data[0] = (float)recovery_metrics_.total_recovery_attempts;
            recovery_msg_.data.data[1] = (float)recovery_metrics_.successful_recoveries;
            recovery_msg_.data.data[2] = (float)recovery_metrics_.failed_recoveries;
            recovery_msg_.data.data[3] = recovery_metrics_.success_rate;
            recovery_msg_.data.data[4] = (float)recovery_metrics_.average_recovery_time_ms;
            recovery_msg_.data.data[5] = (float)recovery_metrics_.communication_recoveries;
            recovery_msg_.data.data[6] = (float)recovery_metrics_.hardware_recoveries;
            recovery_msg_.data.data[7] = (float)recovery_metrics_.sensor_recoveries;
            recovery_msg_.data.data[8] = (float)recovery_metrics_.motor_recoveries;
            recovery_msg_.data.data[9] = (float)recovery_metrics_.memory_recoveries;
            
            rcl_publish(recovery_metrics_publisher_, &recovery_msg_, NULL);
        }
        
        xSemaphoreGive(error_mutex_);
    }
}

void ErrorLogger::update() {
    uint32_t now = millis();
    
    // Analyze patterns periodically
    if (pattern_analysis_enabled_ && (now - last_pattern_analysis_ >= PATTERN_ANALYSIS_INTERVAL_MS)) {
        analyzeErrorPatterns();
        last_pattern_analysis_ = now;
    }
    
    // Publish logs periodically
    if (ros_publishing_enabled_ && (now - last_log_publish_ >= LOG_PUBLISH_INTERVAL_MS)) {
        publishErrorLogs();
        publishRecoveryMetrics();
        last_log_publish_ = now;
    }
}

void ErrorLogger::analyzePatterns() {
    analyzeErrorPatterns();
}

bool ErrorLogger::isRecurringError(ErrorCategory category, uint16_t error_code) {
    size_t pattern_index = findExistingPattern(category, error_code);
    if (pattern_index < MAX_ERROR_PATTERNS) {
        return error_patterns_[pattern_index].is_recurring;
    }
    return false;
}

float ErrorLogger::getErrorFrequency(ErrorCategory category, uint16_t error_code) {
    size_t pattern_index = findExistingPattern(category, error_code);
    if (pattern_index < MAX_ERROR_PATTERNS) {
        return error_patterns_[pattern_index].frequency_per_minute;
    }
    return 0.0f;
}

RecoveryMetrics ErrorLogger::getRecoveryMetrics() const {
    return recovery_metrics_;
}

size_t ErrorLogger::getErrorCount(ErrorSeverity min_severity) const {
    size_t count = 0;
    for (size_t i = 0; i < event_count_; i++) {
        if (error_events_[i].severity >= min_severity) {
            count++;
        }
    }
    return count;
}

size_t ErrorLogger::getErrorCount(ErrorCategory category, ErrorSeverity min_severity) const {
    size_t count = 0;
    for (size_t i = 0; i < event_count_; i++) {
        if (error_events_[i].category == category && error_events_[i].severity >= min_severity) {
            count++;
        }
    }
    return count;
}

ErrorEvent ErrorLogger::getRecentError(size_t index) const {
    if (index < event_count_) {
        size_t actual_index = (next_event_index_ + MAX_ERROR_EVENTS - 1 - index) % MAX_ERROR_EVENTS;
        return error_events_[actual_index];
    }
    return ErrorEvent();
}

ErrorPattern ErrorLogger::getErrorPattern(size_t index) const {
    if (index < pattern_count_) {
        return error_patterns_[index];
    }
    return ErrorPattern();
}

void ErrorLogger::publishCurrentLogs() {
    publishErrorLogs();
}

void ErrorLogger::publishCurrentMetrics() {
    publishRecoveryMetrics();
}

void ErrorLogger::clearErrorHistory() {
    if (error_mutex_ != NULL && xSemaphoreTake(error_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        event_count_ = 0;
        next_event_index_ = 0;
        
        for (size_t i = 0; i < MAX_ERROR_EVENTS; i++) {
            error_events_[i] = ErrorEvent();
        }
        
        xSemaphoreGive(error_mutex_);
    }
}

void ErrorLogger::clearRecoveryMetrics() {
    if (error_mutex_ != NULL && xSemaphoreTake(error_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        recovery_metrics_ = RecoveryMetrics();
        xSemaphoreGive(error_mutex_);
    }
}

void ErrorLogger::clearErrorPatterns() {
    if (error_mutex_ != NULL && xSemaphoreTake(error_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        pattern_count_ = 0;
        
        for (size_t i = 0; i < MAX_ERROR_PATTERNS; i++) {
            error_patterns_[i] = ErrorPattern();
        }
        
        xSemaphoreGive(error_mutex_);
    }
}

size_t ErrorLogger::findExistingEvent(ErrorCategory category, uint16_t error_code) {
    for (size_t i = 0; i < event_count_; i++) {
        if (error_events_[i].category == category && error_events_[i].error_code == error_code) {
            return i;
        }
    }
    return MAX_ERROR_EVENTS;  // Not found
}

size_t ErrorLogger::findExistingPattern(ErrorCategory category, uint16_t error_code) {
    for (size_t i = 0; i < pattern_count_; i++) {
        if (error_patterns_[i].category == category && error_patterns_[i].error_code == error_code) {
            return i;
        }
    }
    return MAX_ERROR_PATTERNS;  // Not found
}

const char* ErrorLogger::severityToString(ErrorSeverity severity) {
    switch (severity) {
        case ErrorSeverity::DEBUG: return "DEBUG";
        case ErrorSeverity::INFO: return "INFO";
        case ErrorSeverity::WARNING: return "WARNING";
        case ErrorSeverity::ERROR: return "ERROR";
        case ErrorSeverity::CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

const char* ErrorLogger::categoryToString(ErrorCategory category) {
    switch (category) {
        case ErrorCategory::SYSTEM: return "SYSTEM";
        case ErrorCategory::COMMUNICATION: return "COMMUNICATION";
        case ErrorCategory::HARDWARE: return "HARDWARE";
        case ErrorCategory::SENSOR: return "SENSOR";
        case ErrorCategory::MOTOR: return "MOTOR";
        case ErrorCategory::MEMORY: return "MEMORY";
        case ErrorCategory::TIMING: return "TIMING";
        case ErrorCategory::RECOVERY: return "RECOVERY";
        default: return "UNKNOWN";
    }
}