#include "system_health_dashboard.h"
#include "system_performance_monitor.h"
#include "error_logger.h"
#include <rclc/rclc.h>
#include <string.h>
#include <Arduino.h>
#include <algorithm>

// Global instance
SystemHealthDashboard g_health_dashboard;

SystemHealthDashboard::SystemHealthDashboard()
    : health_status_publisher_(nullptr)
    , diagnostic_publisher_(nullptr)
    , performance_publisher_(nullptr)
    , performance_monitor_(nullptr)
    , error_logger_(nullptr)
    , debug_manager_(nullptr)
    , last_health_publish_(0)
    , last_diagnostic_publish_(0)
    , last_performance_publish_(0)
    , last_health_update_(0)
    , diagnostic_count_(0)
    , dashboard_enabled_(true)
    , min_diagnostic_level_(SystemHealthStatus::OK)
{
    dashboard_mutex_ = xSemaphoreCreateMutex();
    
    // Initialize diagnostic entries
    for (size_t i = 0; i < MAX_DIAGNOSTIC_ENTRIES; i++) {
        diagnostic_entries_[i].active = false;
        diagnostic_entries_[i].name[0] = '\0';
        diagnostic_entries_[i].message[0] = '\0';
        diagnostic_entries_[i].level = SystemHealthStatus::OK;
        diagnostic_entries_[i].value = 0.0f;
        diagnostic_entries_[i].timestamp = 0;
    }
}

SystemHealthDashboard::~SystemHealthDashboard() {
    shutdown();
    if (dashboard_mutex_ != NULL) {
        vSemaphoreDelete(dashboard_mutex_);
    }
}

bool SystemHealthDashboard::initialize(rcl_node_t* node, SystemPerformanceMonitor* perf_monitor, 
                                      ErrorLogger* error_logger, DebugManager* debug_manager) {
    if (node == NULL) return false;
    
    // Store component references
    performance_monitor_ = perf_monitor;
    error_logger_ = error_logger;
    debug_manager_ = debug_manager;
    
    // Initialize health status publisher
    health_status_publisher_ = (rcl_publisher_t*)malloc(sizeof(rcl_publisher_t));
    if (health_status_publisher_ == NULL) return false;
    
    rcl_ret_t ret = rclc_publisher_init_best_effort(
        health_status_publisher_, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "system_health"
    );
    
    if (ret != RCL_RET_OK) {
        free(health_status_publisher_);
        health_status_publisher_ = nullptr;
        return false;
    }
    
    // Initialize diagnostic publisher
    diagnostic_publisher_ = (rcl_publisher_t*)malloc(sizeof(rcl_publisher_t));
    if (diagnostic_publisher_ == NULL) {
        rcl_publisher_fini(health_status_publisher_, node);
        free(health_status_publisher_);
        health_status_publisher_ = nullptr;
        return false;
    }
    
    ret = rclc_publisher_init_best_effort(
        diagnostic_publisher_, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),  // Simplified for ESP32
        "system_diagnostics"
    );
    
    if (ret != RCL_RET_OK) {
        rcl_publisher_fini(health_status_publisher_, node);
        free(health_status_publisher_);
        free(diagnostic_publisher_);
        health_status_publisher_ = nullptr;
        diagnostic_publisher_ = nullptr;
        return false;
    }
    
    // Initialize performance publisher
    performance_publisher_ = (rcl_publisher_t*)malloc(sizeof(rcl_publisher_t));
    if (performance_publisher_ == NULL) {
        rcl_publisher_fini(health_status_publisher_, node);
        rcl_publisher_fini(diagnostic_publisher_, node);
        free(health_status_publisher_);
        free(diagnostic_publisher_);
        health_status_publisher_ = nullptr;
        diagnostic_publisher_ = nullptr;
        return false;
    }
    
    ret = rclc_publisher_init_best_effort(
        performance_publisher_, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "system_performance"
    );
    
    if (ret != RCL_RET_OK) {
        rcl_publisher_fini(health_status_publisher_, node);
        rcl_publisher_fini(diagnostic_publisher_, node);
        free(health_status_publisher_);
        free(diagnostic_publisher_);
        free(performance_publisher_);
        health_status_publisher_ = nullptr;
        diagnostic_publisher_ = nullptr;
        performance_publisher_ = nullptr;
        return false;
    }
    
    // Initialize ROS messages
    if (!initializeROSMessages()) {
        shutdown();
        return false;
    }
    
    return true;
}

void SystemHealthDashboard::shutdown() {
    cleanupROSMessages();
    
    if (health_status_publisher_ != nullptr) {
        free(health_status_publisher_);
        health_status_publisher_ = nullptr;
    }
    
    if (diagnostic_publisher_ != nullptr) {
        free(diagnostic_publisher_);
        diagnostic_publisher_ = nullptr;
    }
    
    if (performance_publisher_ != nullptr) {
        free(performance_publisher_);
        performance_publisher_ = nullptr;
    }
}

bool SystemHealthDashboard::initializeROSMessages() {
    // Initialize health status message
    health_msg_.data.size = 0;
    health_msg_.data.capacity = 15;  // Various health indicators
    health_msg_.data.data = (float*)malloc(15 * sizeof(float));
    if (health_msg_.data.data == NULL) return false;
    
    // Initialize diagnostic message (simplified as Float32MultiArray)
    diagnostic_msg_.data.size = 0;
    diagnostic_msg_.data.capacity = 20;  // Diagnostic data
    diagnostic_msg_.data.data = (float*)malloc(20 * sizeof(float));
    if (diagnostic_msg_.data.data == NULL) {
        free(health_msg_.data.data);
        health_msg_.data.data = NULL;
        return false;
    }
    
    // Initialize performance message
    performance_msg_.data.size = 0;
    performance_msg_.data.capacity = 25;  // Performance metrics
    performance_msg_.data.data = (float*)malloc(25 * sizeof(float));
    if (performance_msg_.data.data == NULL) {
        free(health_msg_.data.data);
        free(diagnostic_msg_.data.data);
        health_msg_.data.data = NULL;
        diagnostic_msg_.data.data = NULL;
        return false;
    }
    
    return true;
}

void SystemHealthDashboard::cleanupROSMessages() {
    if (health_msg_.data.data != NULL) {
        free(health_msg_.data.data);
        health_msg_.data.data = NULL;
    }
    
    if (diagnostic_msg_.data.data != NULL) {
        free(diagnostic_msg_.data.data);
        diagnostic_msg_.data.data = NULL;
    }
    
    if (performance_msg_.data.data != NULL) {
        free(performance_msg_.data.data);
        performance_msg_.data.data = NULL;
    }
}

void SystemHealthDashboard::setDebugLevel(DebugLevel level) {
    config_.level = level;
    
    // Adjust configuration based on debug level
    switch (level) {
        case DebugLevel::MINIMAL:
            config_.enable_performance_metrics = false;
            config_.enable_timing_diagnostics = false;
            config_.enable_memory_diagnostics = false;
            config_.publish_interval_ms = 5000;  // 0.2Hz
            break;
            
        case DebugLevel::BASIC:
            config_.enable_performance_metrics = true;
            config_.enable_timing_diagnostics = false;
            config_.enable_memory_diagnostics = false;
            config_.publish_interval_ms = 2000;  // 0.5Hz
            break;
            
        case DebugLevel::DETAILED:
            config_.enable_performance_metrics = true;
            config_.enable_timing_diagnostics = true;
            config_.enable_memory_diagnostics = true;
            config_.publish_interval_ms = 1000;  // 1Hz
            break;
            
        case DebugLevel::VERBOSE:
            config_.enable_performance_metrics = true;
            config_.enable_timing_diagnostics = true;
            config_.enable_memory_diagnostics = true;
            config_.enable_communication_diagnostics = true;
            config_.publish_interval_ms = 500;   // 2Hz
            break;
            
        case DebugLevel::DEVELOPER:
            config_.enable_performance_metrics = true;
            config_.enable_timing_diagnostics = true;
            config_.enable_memory_diagnostics = true;
            config_.enable_communication_diagnostics = true;
            config_.publish_interval_ms = 250;   // 4Hz
            break;
    }
}

DebugLevel SystemHealthDashboard::getDebugLevel() const {
    return config_.level;
}

void SystemHealthDashboard::setDiagnosticConfig(const DiagnosticConfig& config) {
    if (dashboard_mutex_ != NULL && xSemaphoreTake(dashboard_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        config_ = config;
        xSemaphoreGive(dashboard_mutex_);
    }
}

DiagnosticConfig SystemHealthDashboard::getDiagnosticConfig() const {
    return config_;
}

void SystemHealthDashboard::setPublishInterval(uint32_t interval_ms) {
    config_.publish_interval_ms = interval_ms;
}

void SystemHealthDashboard::enablePerformanceMetrics(bool enable) {
    config_.enable_performance_metrics = enable;
}

void SystemHealthDashboard::enableErrorReporting(bool enable) {
    config_.enable_error_reporting = enable;
}

void SystemHealthDashboard::enableHealthMonitoring(bool enable) {
    config_.enable_health_monitoring = enable;
}

void SystemHealthDashboard::enableTimingDiagnostics(bool enable) {
    config_.enable_timing_diagnostics = enable;
}

void SystemHealthDashboard::enableMemoryDiagnostics(bool enable) {
    config_.enable_memory_diagnostics = enable;
}

void SystemHealthDashboard::enableCommunicationDiagnostics(bool enable) {
    config_.enable_communication_diagnostics = enable;
}

void SystemHealthDashboard::update() {
    if (!dashboard_enabled_) return;
    
    uint32_t now = millis();
    
    // Update system health
    if (now - last_health_update_ >= HEALTH_UPDATE_INTERVAL_MS) {
        updateSystemHealth();
        last_health_update_ = now;
    }
    
    // Publish health status
    if (config_.enable_health_monitoring && 
        (now - last_health_publish_ >= config_.publish_interval_ms)) {
        publishHealthStatus();
        last_health_publish_ = now;
    }
    
    // Publish diagnostics
    if ((config_.enable_timing_diagnostics || config_.enable_memory_diagnostics || 
         config_.enable_communication_diagnostics) &&
        (now - last_diagnostic_publish_ >= config_.publish_interval_ms)) {
        publishDiagnostics();
        last_diagnostic_publish_ = now;
    }
    
    // Publish performance metrics
    if (config_.enable_performance_metrics && 
        (now - last_performance_publish_ >= config_.publish_interval_ms)) {
        publishPerformanceMetrics();
        last_performance_publish_ = now;
    }
}

SystemHealthStatus SystemHealthDashboard::getSystemHealthStatus() const {
    if (dashboard_mutex_ != NULL && xSemaphoreTake(dashboard_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        SystemHealthStatus status_copy = current_status_;
        xSemaphoreGive(dashboard_mutex_);
        return status_copy;
    }
    return current_status_;
}

bool SystemHealthDashboard::isSystemHealthy() const {
    return current_status_.overall_status == SystemHealthStatus::OK ||
           current_status_.overall_status == SystemHealthStatus::WARN;
}

float SystemHealthDashboard::getHealthScore() const {
    return current_status_.health_score;
}

void SystemHealthDashboard::updateSystemHealth() {
    if (dashboard_mutex_ != NULL && xSemaphoreTake(dashboard_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        
        // Get system state from performance monitor
        if (performance_monitor_ != nullptr) {
            SystemState sys_state = performance_monitor_->getSystemState();
            
            // Update individual status components
            current_status_.connection_status = (sys_state.connection_state == SystemState::CONNECTED) ? 
                SystemHealthStatus::OK : SystemHealthStatus::ERROR;
            
            current_status_.motor_status = sys_state.motors_healthy ? 
                SystemHealthStatus::OK : SystemHealthStatus::ERROR;
            
            current_status_.sensor_status = sys_state.sensors_healthy ? 
                SystemHealthStatus::OK : SystemHealthStatus::WARN;
            
            current_status_.memory_status = sys_state.memory_healthy ? 
                SystemHealthStatus::OK : SystemHealthStatus::WARN;
            
            current_status_.timing_status = sys_state.timing_healthy ? 
                SystemHealthStatus::OK : SystemHealthStatus::WARN;
            
            current_status_.spi_status = sys_state.spi_healthy ? 
                SystemHealthStatus::OK : SystemHealthStatus::ERROR;
            
            // Update health score and uptime
            current_status_.health_score = sys_state.overall_health_score;
            current_status_.uptime_seconds = sys_state.uptime_ms / 1000;
        }
        
        // Calculate overall status
        current_status_.overall_status = calculateOverallStatus();
        
        // Update status message
        updateStatusMessage();
        
        xSemaphoreGive(dashboard_mutex_);
    }
}

void SystemHealthDashboard::publishHealthStatus() {
    if (!dashboard_enabled_ || health_status_publisher_ == NULL) return;
    
    if (dashboard_mutex_ != NULL && xSemaphoreTake(dashboard_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        
        if (health_msg_.data.data != NULL) {
            health_msg_.data.size = 10;
            health_msg_.data.data[0] = (float)current_status_.overall_status;
            health_msg_.data.data[1] = (float)current_status_.connection_status;
            health_msg_.data.data[2] = (float)current_status_.motor_status;
            health_msg_.data.data[3] = (float)current_status_.sensor_status;
            health_msg_.data.data[4] = (float)current_status_.memory_status;
            health_msg_.data.data[5] = (float)current_status_.timing_status;
            health_msg_.data.data[6] = (float)current_status_.spi_status;
            health_msg_.data.data[7] = current_status_.health_score;
            health_msg_.data.data[8] = (float)current_status_.uptime_seconds;
            health_msg_.data.data[9] = (float)millis();  // Timestamp
            
            rcl_publish(health_status_publisher_, &health_msg_, NULL);
        }
        
        xSemaphoreGive(dashboard_mutex_);
    }
}

void SystemHealthDashboard::publishDiagnostics() {
    if (!dashboard_enabled_ || diagnostic_publisher_ == NULL) return;
    
    if (dashboard_mutex_ != NULL && xSemaphoreTake(dashboard_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        
        if (diagnostic_msg_.data.data != NULL) {
            // Publish diagnostic entries as float array
            size_t data_index = 0;
            diagnostic_msg_.data.size = 0;
            
            for (size_t i = 0; i < diagnostic_count_ && i < MAX_DIAGNOSTIC_ENTRIES && data_index < 18; i++) {
                if (diagnostic_entries_[i].active && 
                    diagnostic_entries_[i].level >= min_diagnostic_level_) {
                    
                    diagnostic_msg_.data.data[data_index++] = (float)diagnostic_entries_[i].level;
                    diagnostic_msg_.data.data[data_index++] = diagnostic_entries_[i].value;
                    diagnostic_msg_.data.data[data_index++] = (float)diagnostic_entries_[i].timestamp;
                    diagnostic_msg_.data.size += 3;
                }
            }
            
            if (diagnostic_msg_.data.size > 0) {
                rcl_publish(diagnostic_publisher_, &diagnostic_msg_, NULL);
            }
        }
        
        xSemaphoreGive(dashboard_mutex_);
    }
}

void SystemHealthDashboard::publishPerformanceMetrics() {
    if (!dashboard_enabled_ || performance_publisher_ == NULL) return;
    
    if (dashboard_mutex_ != NULL && xSemaphoreTake(dashboard_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        
        if (performance_msg_.data.data != NULL && performance_monitor_ != nullptr) {
            SystemState sys_state = performance_monitor_->getSystemState();
            
            performance_msg_.data.size = 15;
            performance_msg_.data.data[0] = sys_state.cpu_usage_percent;
            performance_msg_.data.data[1] = (float)(sys_state.free_memory_bytes / 1024);  // KB
            performance_msg_.data.data[2] = sys_state.average_loop_time_ms;
            performance_msg_.data.data[3] = sys_state.max_loop_time_ms;
            performance_msg_.data.data[4] = (float)sys_state.loop_overruns;
            performance_msg_.data.data[5] = (float)sys_state.twist_callbacks_per_sec;
            performance_msg_.data.data[6] = (float)sys_state.odom_publishes_per_sec;
            performance_msg_.data.data[7] = (float)sys_state.debug_messages_per_sec;
            performance_msg_.data.data[8] = (float)sys_state.spi_errors;
            performance_msg_.data.data[9] = (float)sys_state.connection_drops;
            performance_msg_.data.data[10] = (float)sys_state.sensor_failures;
            performance_msg_.data.data[11] = (float)sys_state.memory_allocation_failures;
            performance_msg_.data.data[12] = sys_state.overall_health_score;
            performance_msg_.data.data[13] = (float)sys_state.uptime_ms;
            performance_msg_.data.data[14] = (float)millis();  // Timestamp
            
            rcl_publish(performance_publisher_, &performance_msg_, NULL);
        }
        
        xSemaphoreGive(dashboard_mutex_);
    }
}

SystemHealthStatus::Level SystemHealthDashboard::calculateOverallStatus() {
    // Determine overall status based on individual components
    SystemHealthStatus::Level worst_status = SystemHealthStatus::OK;
    
    // Check critical components first
    if (current_status_.connection_status == SystemHealthStatus::ERROR ||
        current_status_.motor_status == SystemHealthStatus::ERROR ||
        current_status_.spi_status == SystemHealthStatus::ERROR) {
        worst_status = SystemHealthStatus::ERROR;
    }
    // Check warning conditions
    else if (current_status_.sensor_status == SystemHealthStatus::WARN ||
             current_status_.memory_status == SystemHealthStatus::WARN ||
             current_status_.timing_status == SystemHealthStatus::WARN) {
        worst_status = SystemHealthStatus::WARN;
    }
    
    // Check health score
    if (current_status_.health_score < 0.5f) {
        worst_status = SystemHealthStatus::ERROR;
    } else if (current_status_.health_score < 0.7f && worst_status == SystemHealthStatus::OK) {
        worst_status = SystemHealthStatus::WARN;
    }
    
    return worst_status;
}

void SystemHealthDashboard::updateStatusMessage() {
    switch (current_status_.overall_status) {
        case SystemHealthStatus::OK:
            snprintf(current_status_.status_message, sizeof(current_status_.status_message),
                    "System healthy (%.1f%% health)", current_status_.health_score * 100);
            break;
            
        case SystemHealthStatus::WARN:
            snprintf(current_status_.status_message, sizeof(current_status_.status_message),
                    "System warnings detected (%.1f%% health)", current_status_.health_score * 100);
            break;
            
        case SystemHealthStatus::ERROR:
            snprintf(current_status_.status_message, sizeof(current_status_.status_message),
                    "System errors detected (%.1f%% health)", current_status_.health_score * 100);
            break;
            
        case SystemHealthStatus::STALE:
            strcpy(current_status_.status_message, "System status unknown");
            break;
    }
}

void SystemHealthDashboard::addDiagnosticEntry(const char* name, SystemHealthStatus::Level level, 
                                              const char* message, float value) {
    if (dashboard_mutex_ != NULL && xSemaphoreTake(dashboard_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        
        // Find existing entry or create new one
        size_t entry_index = diagnostic_count_;
        for (size_t i = 0; i < diagnostic_count_; i++) {
            if (strcmp(diagnostic_entries_[i].name, name) == 0) {
                entry_index = i;
                break;
            }
        }
        
        // Create new entry if needed
        if (entry_index >= MAX_DIAGNOSTIC_ENTRIES) {
            xSemaphoreGive(dashboard_mutex_);
            return;  // No space
        }
        
        if (entry_index == diagnostic_count_ && diagnostic_count_ < MAX_DIAGNOSTIC_ENTRIES) {
            diagnostic_count_++;
        }
        
        // Update entry
        DiagnosticEntry& entry = diagnostic_entries_[entry_index];
        strncpy(entry.name, name, sizeof(entry.name) - 1);
        entry.name[sizeof(entry.name) - 1] = '\0';
        entry.level = level;
        strncpy(entry.message, message, sizeof(entry.message) - 1);
        entry.message[sizeof(entry.message) - 1] = '\0';
        entry.value = value;
        entry.timestamp = millis();
        entry.active = true;
        
        xSemaphoreGive(dashboard_mutex_);
    }
}

void SystemHealthDashboard::reportSystemStatus(const char* component, SystemHealthStatus::Level level, const char* message) {
    addDiagnosticEntry(component, level, message, 0.0f);
}

void SystemHealthDashboard::reportPerformanceMetric(const char* metric_name, float value, SystemHealthStatus::Level level) {
    addDiagnosticEntry(metric_name, level, "performance_metric", value);
}

void SystemHealthDashboard::reportError(const char* error_source, const char* error_message) {
    addDiagnosticEntry(error_source, SystemHealthStatus::ERROR, error_message, 0.0f);
}

void SystemHealthDashboard::reportWarning(const char* warning_source, const char* warning_message) {
    addDiagnosticEntry(warning_source, SystemHealthStatus::WARN, warning_message, 0.0f);
}

void SystemHealthDashboard::reportInfo(const char* info_source, const char* info_message) {
    addDiagnosticEntry(info_source, SystemHealthStatus::OK, info_message, 0.0f);
}

void SystemHealthDashboard::publishCurrentHealth() {
    publishHealthStatus();
}

void SystemHealthDashboard::publishCurrentDiagnostics() {
    publishDiagnostics();
}

void SystemHealthDashboard::publishCurrentPerformance() {
    publishPerformanceMetrics();
}

void SystemHealthDashboard::publishAll() {
    publishHealthStatus();
    publishDiagnostics();
    publishPerformanceMetrics();
}

void SystemHealthDashboard::setMinimumDiagnosticLevel(SystemHealthStatus::Level min_level) {
    min_diagnostic_level_ = min_level;
}

SystemHealthStatus::Level SystemHealthDashboard::getMinimumDiagnosticLevel() const {
    return min_diagnostic_level_;
}

void SystemHealthDashboard::enableDashboard(bool enable) {
    dashboard_enabled_ = enable;
}

bool SystemHealthDashboard::isDashboardEnabled() const {
    return dashboard_enabled_;
}

void SystemHealthDashboard::resetDashboard() {
    if (dashboard_mutex_ != NULL && xSemaphoreTake(dashboard_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        clearDiagnosticEntries();
        current_status_ = SystemHealthStatus();
        xSemaphoreGive(dashboard_mutex_);
    }
}

void SystemHealthDashboard::clearDiagnosticEntries() {
    diagnostic_count_ = 0;
    for (size_t i = 0; i < MAX_DIAGNOSTIC_ENTRIES; i++) {
        diagnostic_entries_[i].active = false;
    }
}

const char* SystemHealthDashboard::levelToString(SystemHealthStatus::Level level) {
    switch (level) {
        case SystemHealthStatus::OK: return "OK";
        case SystemHealthStatus::WARN: return "WARN";
        case SystemHealthStatus::ERROR: return "ERROR";
        case SystemHealthStatus::STALE: return "STALE";
        default: return "UNKNOWN";
    }
}