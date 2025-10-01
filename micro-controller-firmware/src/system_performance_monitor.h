#ifndef SYSTEM_PERFORMANCE_MONITOR_H
#define SYSTEM_PERFORMANCE_MONITOR_H

#include <stdint.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Forward declarations
class ConnectionManager;
class Car;
class SensorCache;

struct SystemState {
    // Connection status
    enum ConnectionState {
        DISCONNECTED = 0,
        CONNECTING = 1,
        CONNECTED = 2,
        DEGRADED = 3
    } connection_state;
    
    // System initialization status
    bool car_initialized;
    bool ros_initialized;
    bool sensors_initialized;
    bool system_components_initialized;
    
    // Health indicators
    bool motors_healthy;
    bool sensors_healthy;
    bool memory_healthy;
    bool timing_healthy;
    bool spi_healthy;
    
    // Performance metrics
    uint32_t uptime_ms;
    float cpu_usage_percent;
    size_t free_memory_bytes;
    size_t total_memory_bytes;
    
    // Timing metrics
    float average_loop_time_ms;
    float max_loop_time_ms;
    uint32_t loop_overruns;
    
    // Communication metrics
    uint32_t twist_callbacks_per_sec;
    uint32_t odom_publishes_per_sec;
    uint32_t debug_messages_per_sec;
    
    // Error counters
    uint32_t spi_errors;
    uint32_t connection_drops;
    uint32_t sensor_failures;
    uint32_t memory_allocation_failures;
    
    // System health score (0.0 to 1.0)
    float overall_health_score;
    
    // Timestamps
    uint32_t last_update_time;
    uint32_t system_start_time;
    
    SystemState() {
        connection_state = DISCONNECTED;
        car_initialized = false;
        ros_initialized = false;
        sensors_initialized = false;
        system_components_initialized = false;
        motors_healthy = false;
        sensors_healthy = false;
        memory_healthy = false;
        timing_healthy = false;
        spi_healthy = false;
        uptime_ms = 0;
        cpu_usage_percent = 0.0f;
        free_memory_bytes = 0;
        total_memory_bytes = 0;
        average_loop_time_ms = 0.0f;
        max_loop_time_ms = 0.0f;
        loop_overruns = 0;
        twist_callbacks_per_sec = 0;
        odom_publishes_per_sec = 0;
        debug_messages_per_sec = 0;
        spi_errors = 0;
        connection_drops = 0;
        sensor_failures = 0;
        memory_allocation_failures = 0;
        overall_health_score = 0.0f;
        last_update_time = 0;
        system_start_time = 0;
    }
};

struct SystemHealthThresholds {
    float min_memory_mb;
    float max_cpu_usage_percent;
    float max_loop_time_ms;
    uint32_t max_spi_errors_per_minute;
    uint32_t max_connection_drops_per_hour;
    float min_health_score;
    
    SystemHealthThresholds() {
        min_memory_mb = 0.05f;  // 50KB minimum
        max_cpu_usage_percent = 85.0f;
        max_loop_time_ms = 250.0f;
        max_spi_errors_per_minute = 5;
        max_connection_drops_per_hour = 3;
        min_health_score = 0.7f;
    }
};

class SystemPerformanceMonitor {
private:
    static const uint32_t UPDATE_INTERVAL_MS = 1000;  // 1Hz updates
    static const uint32_t HEALTH_CHECK_INTERVAL_MS = 500;  // 2Hz health checks
    static const uint32_t ALERT_COOLDOWN_MS = 30000;  // 30 second alert cooldown
    
    // System state
    SystemState current_state_;
    SystemHealthThresholds thresholds_;
    
    // Dependencies
    ConnectionManager* connection_manager_;
    Car* car_;
    SensorCache* sensor_cache_;
    
    // Timing
    uint32_t last_update_time_;
    uint32_t last_health_check_;
    uint32_t last_alert_time_;
    
    // Rate calculation
    struct RateCounters {
        uint32_t twist_count;
        uint32_t odom_count;
        uint32_t debug_count;
        uint32_t last_rate_calc_time;
    } rate_counters_;
    
    // Thread safety
    SemaphoreHandle_t state_mutex_;
    
    // CPU usage tracking
    struct CPUTracker {
        uint32_t last_idle_time;
        uint32_t last_total_time;
        uint32_t last_measurement_time;
    } cpu_tracker_;
    
    // Memory tracking
    struct MemoryTracker {
        size_t baseline_free_memory;
        size_t min_free_memory;
        uint32_t allocation_failures;
    } memory_tracker_;
    
    // Helper methods
    void updateSystemMetrics();
    void updateHealthIndicators();
    void calculateRates();
    void updateCPUUsage();
    void updateMemoryUsage();
    void updateTimingMetrics();
    void updateConnectionMetrics();
    void updateHardwareHealth();
    float calculateHealthScore();
    bool checkHealthThresholds();
    void triggerHealthAlert(const char* alert_type, float value, float threshold);

public:
    SystemPerformanceMonitor();
    ~SystemPerformanceMonitor();
    
    // Initialization
    bool initialize(ConnectionManager* conn_mgr, Car* car, SensorCache* sensor_cache);
    void shutdown();
    
    // Configuration
    void setHealthThresholds(const SystemHealthThresholds& thresholds);
    SystemHealthThresholds getHealthThresholds() const;
    
    // Main update function
    void update();
    
    // State access
    SystemState getSystemState() const;
    bool isSystemHealthy() const;
    float getHealthScore() const;
    
    // Event recording
    void recordTwistCallback();
    void recordOdomPublish();
    void recordDebugMessage();
    void recordLoopTiming(float loop_time_ms);
    void recordSPIError();
    void recordConnectionDrop();
    void recordSensorFailure();
    void recordMemoryAllocationFailure();
    
    // System initialization tracking
    void setCarInitialized(bool initialized);
    void setROSInitialized(bool initialized);
    void setSensorsInitialized(bool initialized);
    void setSystemComponentsInitialized(bool initialized);
    
    // Health monitoring
    bool isMemoryHealthy() const;
    bool isTimingHealthy() const;
    bool areMotorsHealthy() const;
    bool areSensorsHealthy() const;
    bool isSPIHealthy() const;
    
    // Performance metrics
    uint32_t getUptime() const;
    float getCPUUsage() const;
    size_t getFreeMemory() const;
    float getAverageLoopTime() const;
    uint32_t getErrorCount(const char* error_type) const;
    
    // Alerting
    void enableHealthAlerting(bool enable);
    bool isHealthAlertingEnabled() const;
    
private:
    bool health_alerting_enabled_;
};

#endif // SYSTEM_PERFORMANCE_MONITOR_H