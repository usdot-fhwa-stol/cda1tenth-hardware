#ifndef DEBUG_MANAGER_H
#define DEBUG_MANAGER_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <rcl/rcl.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <atomic>

enum class DebugLevel : uint8_t {
    ERROR = 0,
    WARN = 1,
    INFO = 2,
    DEBUG = 3
};

struct PerformanceMetrics {
    // Callback frequencies
    std::atomic<uint32_t> twist_callbacks_per_sec{0};
    std::atomic<uint32_t> odom_publishes_per_sec{0};
    std::atomic<uint32_t> control_loop_frequency{0};
    
    // Timing metrics
    std::atomic<float> average_loop_time_ms{0.0f};
    std::atomic<float> max_loop_time_ms{0.0f};
    std::atomic<uint32_t> loop_overruns{0};
    
    // Error counters
    std::atomic<uint32_t> spi_errors{0};
    std::atomic<uint32_t> connection_drops{0};
    std::atomic<uint32_t> command_queue_overflows{0};
    std::atomic<uint32_t> sensor_read_failures{0};
    
    // System health
    std::atomic<float> cpu_usage_percent{0.0f};
    std::atomic<size_t> free_memory_bytes{0};
    std::atomic<uint32_t> uptime_seconds{0};
    
    // Odometry-specific metrics
    std::atomic<uint32_t> odometry_stale_data_events{0};
    std::atomic<uint32_t> odometry_invalid_data_events{0};
    std::atomic<uint32_t> odometry_calculation_errors{0};
    std::atomic<float> odometry_avg_calc_time_ms{0.0f};
    std::atomic<float> odometry_max_calc_time_ms{0.0f};
    
    void reset() {
        twist_callbacks_per_sec.store(0);
        odom_publishes_per_sec.store(0);
        control_loop_frequency.store(0);
        average_loop_time_ms.store(0.0f);
        max_loop_time_ms.store(0.0f);
        loop_overruns.store(0);
        spi_errors.store(0);
        connection_drops.store(0);
        command_queue_overflows.store(0);
        sensor_read_failures.store(0);
        cpu_usage_percent.store(0.0f);
        free_memory_bytes.store(0);
        uptime_seconds.store(0);
        odometry_stale_data_events.store(0);
        odometry_invalid_data_events.store(0);
        odometry_calculation_errors.store(0);
        odometry_avg_calc_time_ms.store(0.0f);
        odometry_max_calc_time_ms.store(0.0f);
    }
};

struct SystemHealthIndicators {
    bool ros_connection_healthy;
    bool motors_healthy;
    bool sensors_healthy;
    bool memory_healthy;
    bool timing_healthy;
    float overall_health_score;  // 0.0 to 1.0
    
    SystemHealthIndicators() : ros_connection_healthy(false), motors_healthy(false),
                              sensors_healthy(false), memory_healthy(false),
                              timing_healthy(false), overall_health_score(0.0f) {}
};

class DebugManager {
private:
    static const size_t MAX_LOG_ENTRIES = 50;
    static const uint32_t METRICS_PUBLISH_INTERVAL_MS = 1000;  // 1Hz
    static const uint32_t HEALTH_CHECK_INTERVAL_MS = 500;      // 2Hz
    static const size_t DEBUG_MSG_SIZE = 20;  // Number of float values in debug message
    
    // ROS publishers
    rcl_publisher_t* debug_publisher;
    rcl_publisher_t* metrics_publisher;
    std_msgs__msg__Float32MultiArray debug_msg;
    std_msgs__msg__Float32MultiArray metrics_msg;
    
    // Debug configuration
    DebugLevel current_level;
    bool ros_logging_enabled;
    bool metrics_collection_enabled;
    
    // Performance tracking
    PerformanceMetrics metrics;
    SystemHealthIndicators health;
    
    // Timing for periodic operations
    uint32_t last_metrics_publish;
    uint32_t last_health_check;
    uint32_t system_start_time;
    
    // Thread safety
    SemaphoreHandle_t debug_mutex;
    
    // Counters for rate calculation
    struct RateCounters {
        uint32_t twist_count;
        uint32_t odom_count;
        uint32_t control_count;
        uint32_t last_rate_calc_time;
    } rate_counters;
    
    // Helper functions
    void updateSystemHealth();
    void calculateRates();
    void publishMetrics();
    bool initializeROSMessages();
    void cleanupROSMessages();
    float calculateOverallHealthScore();

public:
    DebugManager();
    ~DebugManager();
    
    // Initialization
    bool initialize(rcl_node_t* node);
    void shutdown();
    
    // Configuration
    void setLevel(DebugLevel level);
    DebugLevel getLevel() const;
    void enableROSLogging(bool enable);
    void enableMetricsCollection(bool enable);
    
    // Logging functions
    void log(DebugLevel level, const char* message);
    void logError(const char* message);
    void logWarning(const char* message);
    void logInfo(const char* message);
    void logDebug(const char* message);
    
    // Event recording
    void recordTwistCallback();
    void recordOdomPublish();
    void recordControlLoop(float loop_time_ms);
    void recordSPIError();
    void recordConnectionDrop();
    void recordCommandQueueOverflow();
    void recordSensorReadFailure();
    
    // Odometry-specific monitoring
    void recordOdometryCalculationTime(float time_ms);
    void recordOdometryStaleData();
    void recordOdometryInvalidData();
    void recordOdometryError();
    
    // Performance metrics
    const PerformanceMetrics& getMetrics() const;
    const SystemHealthIndicators& getHealthIndicators() const;
    void resetMetrics();
    
    // Periodic update - call from main loop or dedicated thread
    void update();
    
    // Manual publishing
    void publishDebugData();
    void publishCurrentMetrics();
    
    // System monitoring
    void updateMemoryUsage();
    void updateCPUUsage();
    
    // Health scoring
    bool isSystemHealthy() const;
    float getHealthScore() const;
};

// Global debug manager instance
extern DebugManager g_debug_manager;

// Convenience macros
#define DEBUG_ERROR(msg) g_debug_manager.logError(msg)
#define DEBUG_WARN(msg) g_debug_manager.logWarning(msg)
#define DEBUG_INFO(msg) g_debug_manager.logInfo(msg)
#define DEBUG_LOG(msg) g_debug_manager.logDebug(msg)

#define RECORD_TWIST_CALLBACK() g_debug_manager.recordTwistCallback()
#define RECORD_ODOM_PUBLISH() g_debug_manager.recordOdomPublish()
#define RECORD_CONTROL_LOOP(time_ms) g_debug_manager.recordControlLoop(time_ms)
#define RECORD_SPI_ERROR() g_debug_manager.recordSPIError()
#define RECORD_CONNECTION_DROP() g_debug_manager.recordConnectionDrop()

// Odometry monitoring macros
#define RECORD_ODOM_CALC_TIME(time_ms) g_debug_manager.recordOdometryCalculationTime(time_ms)
#define RECORD_ODOM_STALE_DATA() g_debug_manager.recordOdometryStaleData()
#define RECORD_ODOM_INVALID_DATA() g_debug_manager.recordOdometryInvalidData()
#define RECORD_ODOM_ERROR() g_debug_manager.recordOdometryError()

#endif // DEBUG_MANAGER_H