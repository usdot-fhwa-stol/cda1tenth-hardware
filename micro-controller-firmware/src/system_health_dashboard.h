#ifndef SYSTEM_HEALTH_DASHBOARD_H
#define SYSTEM_HEALTH_DASHBOARD_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <rcl/rcl.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "debug_manager.h"  // Include for DebugLevel enum

// Forward declarations
class SystemPerformanceMonitor;
class ErrorLogger;

struct DiagnosticConfig {
    DebugLevel level;
    bool enable_performance_metrics;
    bool enable_error_reporting;
    bool enable_health_monitoring;
    bool enable_timing_diagnostics;
    bool enable_memory_diagnostics;
    bool enable_communication_diagnostics;
    uint32_t publish_interval_ms;
    
    DiagnosticConfig() {
        level = DebugLevel::BASIC;
        enable_performance_metrics = true;
        enable_error_reporting = true;
        enable_health_monitoring = true;
        enable_timing_diagnostics = true;
        enable_memory_diagnostics = true;
        enable_communication_diagnostics = true;
        publish_interval_ms = 1000;  // 1Hz default
    }
};

struct SystemHealthStatus {
    enum Level {
        OK = 0,
        WARN = 1,
        ERROR = 2,
        STALE = 3
    };
    
    Level overall_status;
    Level connection_status;
    Level motor_status;
    Level sensor_status;
    Level memory_status;
    Level timing_status;
    Level spi_status;
    
    float health_score;
    uint32_t uptime_seconds;
    char status_message[128];
    
    SystemHealthStatus() {
        overall_status = STALE;
        connection_status = STALE;
        motor_status = STALE;
        sensor_status = STALE;
        memory_status = STALE;
        timing_status = STALE;
        spi_status = STALE;
        health_score = 0.0f;
        uptime_seconds = 0;
        strcpy(status_message, "System starting...");
    }
};

class SystemHealthDashboard {
private:
    static const size_t MAX_DIAGNOSTIC_ENTRIES = 20;
    static const uint32_t DEFAULT_PUBLISH_INTERVAL_MS = 1000;
    static const uint32_t HEALTH_UPDATE_INTERVAL_MS = 500;
    
    // ROS publishers
    rcl_publisher_t* health_status_publisher_;
    rcl_publisher_t* diagnostic_publisher_;
    rcl_publisher_t* performance_publisher_;
    
    // ROS messages
    std_msgs__msg__Float32MultiArray health_msg_;
    std_msgs__msg__Float32MultiArray diagnostic_msg_;  // Simplified for ESP32
    std_msgs__msg__Float32MultiArray performance_msg_;
    
    // Configuration
    DiagnosticConfig config_;
    
    // System components
    SystemPerformanceMonitor* performance_monitor_;
    ErrorLogger* error_logger_;
    DebugManager* debug_manager_;
    
    // Current status
    SystemHealthStatus current_status_;
    
    // Timing
    uint32_t last_health_publish_;
    uint32_t last_diagnostic_publish_;
    uint32_t last_performance_publish_;
    uint32_t last_health_update_;
    
    // Thread safety
    SemaphoreHandle_t dashboard_mutex_;
    
    // Diagnostic data storage
    struct DiagnosticEntry {
        char name[32];
        SystemHealthStatus::Level level;
        char message[64];
        float value;
        uint32_t timestamp;
        bool active;
    } diagnostic_entries_[MAX_DIAGNOSTIC_ENTRIES];
    
    size_t diagnostic_count_;
    
    // Helper methods
    void updateSystemHealth();
    void publishHealthStatus();
    void publishDiagnostics();
    void publishPerformanceMetrics();
    bool initializeROSMessages();
    void cleanupROSMessages();
    void addDiagnosticEntry(const char* name, SystemHealthStatus::Level level, const char* message, float value);
    void clearDiagnosticEntries();
    SystemHealthStatus::Level calculateOverallStatus();
    void updateStatusMessage();
    const char* levelToString(SystemHealthStatus::Level level);
    void populateDiagnosticMessage();

public:
    SystemHealthDashboard();
    ~SystemHealthDashboard();
    
    // Initialization
    bool initialize(rcl_node_t* node, SystemPerformanceMonitor* perf_monitor, 
                   ErrorLogger* error_logger, DebugManager* debug_manager);
    void shutdown();
    
    // Configuration
    void setDebugLevel(DebugLevel level);
    DebugLevel getDebugLevel() const;
    void setDiagnosticConfig(const DiagnosticConfig& config);
    DiagnosticConfig getDiagnosticConfig() const;
    void setPublishInterval(uint32_t interval_ms);
    
    // Enable/disable specific diagnostics
    void enablePerformanceMetrics(bool enable);
    void enableErrorReporting(bool enable);
    void enableHealthMonitoring(bool enable);
    void enableTimingDiagnostics(bool enable);
    void enableMemoryDiagnostics(bool enable);
    void enableCommunicationDiagnostics(bool enable);
    
    // Main update function
    void update();
    
    // Status access
    SystemHealthStatus getSystemHealthStatus() const;
    bool isSystemHealthy() const;
    float getHealthScore() const;
    
    // Manual publishing
    void publishCurrentHealth();
    void publishCurrentDiagnostics();
    void publishCurrentPerformance();
    void publishAll();
    
    // Diagnostic filtering
    void setMinimumDiagnosticLevel(SystemHealthStatus::Level min_level);
    SystemHealthStatus::Level getMinimumDiagnosticLevel() const;
    
    // Status reporting
    void reportSystemStatus(const char* component, SystemHealthStatus::Level level, const char* message);
    void reportPerformanceMetric(const char* metric_name, float value, SystemHealthStatus::Level level = SystemHealthStatus::OK);
    void reportError(const char* error_source, const char* error_message);
    void reportWarning(const char* warning_source, const char* warning_message);
    void reportInfo(const char* info_source, const char* info_message);
    
    // Dashboard control
    void enableDashboard(bool enable);
    bool isDashboardEnabled() const;
    void resetDashboard();
    
private:
    bool dashboard_enabled_;
    SystemHealthStatus::Level min_diagnostic_level_;
};

// Global dashboard instance
extern SystemHealthDashboard g_health_dashboard;

// Convenience macros for status reporting
#define REPORT_SYSTEM_STATUS(component, level, message) g_health_dashboard.reportSystemStatus(component, level, message)
#define REPORT_PERFORMANCE_METRIC(name, value, level) g_health_dashboard.reportPerformanceMetric(name, value, level)
#define REPORT_ERROR(source, message) g_health_dashboard.reportError(source, message)
#define REPORT_WARNING(source, message) g_health_dashboard.reportWarning(source, message)
#define REPORT_INFO(source, message) g_health_dashboard.reportInfo(source, message)

#endif // SYSTEM_HEALTH_DASHBOARD_H