#include "debug_manager.h"
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>

// Global instance
DebugManager g_debug_manager;

DebugManager::DebugManager() : debug_publisher(nullptr), metrics_publisher(nullptr),
                               current_level(DebugLevel::INFO), ros_logging_enabled(true),
                               metrics_collection_enabled(true), last_metrics_publish(0),
                               last_health_check(0), system_start_time(0) {
    debug_mutex = xSemaphoreCreateMutex();
    
    // Initialize rate counters
    rate_counters.twist_count = 0;
    rate_counters.odom_count = 0;
    rate_counters.control_count = 0;
    rate_counters.last_rate_calc_time = 0;
    
    system_start_time = millis();
}

DebugManager::~DebugManager() {
    shutdown();
    if (debug_mutex != NULL) {
        vSemaphoreDelete(debug_mutex);
    }
}

bool DebugManager::initialize(rcl_node_t* node) {
    if (node == NULL) return false;
    
    // Initialize debug publisher
    debug_publisher = (rcl_publisher_t*)malloc(sizeof(rcl_publisher_t));
    if (debug_publisher == NULL) return false;
    
    rcl_ret_t ret = rclc_publisher_init_best_effort(
        debug_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "debug_log"
    );
    
    if (ret != RCL_RET_OK) {
        free(debug_publisher);
        debug_publisher = nullptr;
        return false;
    }
    
    // Initialize metrics publisher
    metrics_publisher = (rcl_publisher_t*)malloc(sizeof(rcl_publisher_t));
    if (metrics_publisher == NULL) {
        rcl_publisher_fini(debug_publisher, node);
        free(debug_publisher);
        debug_publisher = nullptr;
        return false;
    }
    
    ret = rclc_publisher_init_best_effort(
        metrics_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "system_metrics"
    );
    
    if (ret != RCL_RET_OK) {
        rcl_publisher_fini(debug_publisher, node);
        free(debug_publisher);
        free(metrics_publisher);
        debug_publisher = nullptr;
        metrics_publisher = nullptr;
        return false;
    }
    
    // Initialize ROS messages
    if (!initializeROSMessages()) {
        shutdown();
        return false;
    }
    
    return true;
}

void DebugManager::shutdown() {
    cleanupROSMessages();
    
    if (debug_publisher != nullptr) {
        // Note: rcl_publisher_fini requires node, but we don't have it here
        // This should be called from main cleanup with proper node reference
        free(debug_publisher);
        debug_publisher = nullptr;
    }
    
    if (metrics_publisher != nullptr) {
        free(metrics_publisher);
        metrics_publisher = nullptr;
    }
}

bool DebugManager::initializeROSMessages() {
    // Initialize debug message
    debug_msg.data.size = 0;
    debug_msg.data.capacity = DEBUG_MSG_SIZE;
    debug_msg.data.data = (float*)malloc(DEBUG_MSG_SIZE * sizeof(float));
    if (debug_msg.data.data == NULL) return false;
    
    // Initialize metrics message
    metrics_msg.data.size = 0;
    metrics_msg.data.capacity = DEBUG_MSG_SIZE;
    metrics_msg.data.data = (float*)malloc(DEBUG_MSG_SIZE * sizeof(float));
    if (metrics_msg.data.data == NULL) {
        free(debug_msg.data.data);
        debug_msg.data.data = NULL;
        return false;
    }
    
    return true;
}

void DebugManager::cleanupROSMessages() {
    if (debug_msg.data.data != NULL) {
        free(debug_msg.data.data);
        debug_msg.data.data = NULL;
    }
    
    if (metrics_msg.data.data != NULL) {
        free(metrics_msg.data.data);
        metrics_msg.data.data = NULL;
    }
}

void DebugManager::setLevel(DebugLevel level) {
    current_level = level;
}

DebugLevel DebugManager::getLevel() const {
    return current_level;
}

void DebugManager::enableROSLogging(bool enable) {
    ros_logging_enabled = enable;
}

void DebugManager::enableMetricsCollection(bool enable) {
    metrics_collection_enabled = enable;
}

void DebugManager::log(DebugLevel level, const char* message) {
    if (level > current_level || !ros_logging_enabled) return;
    
    if (debug_mutex == NULL || debug_publisher == NULL) return;
    
    if (xSemaphoreTake(debug_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    
    // Throttle logging to prevent overwhelming the system
    static uint32_t last_log_time = 0;
    uint32_t now = millis();
    
    if (now - last_log_time < 100) {  // Max 10 logs per second
        xSemaphoreGive(debug_mutex);
        return;
    }
    
    if (debug_msg.data.data != NULL) {
        debug_msg.data.size = 4;
        debug_msg.data.data[0] = (float)level;
        debug_msg.data.data[1] = (float)now;
        debug_msg.data.data[2] = (float)strlen(message);  // Message length as identifier
        debug_msg.data.data[3] = (float)(message[0]);     // First character as simple ID
        
        rcl_ret_t ret = rcl_publish(debug_publisher, &debug_msg, NULL);
        if (ret == RCL_RET_OK) {
            last_log_time = now;
        }
    }
    
    xSemaphoreGive(debug_mutex);
}

void DebugManager::logError(const char* message) {
    log(DebugLevel::ERROR, message);
}

void DebugManager::logWarning(const char* message) {
    log(DebugLevel::WARN, message);
}

void DebugManager::logInfo(const char* message) {
    log(DebugLevel::INFO, message);
}

void DebugManager::logDebug(const char* message) {
    log(DebugLevel::DEBUG, message);
}

void DebugManager::recordTwistCallback() {
    if (!metrics_collection_enabled) return;
    rate_counters.twist_count++;
}

void DebugManager::recordOdomPublish() {
    if (!metrics_collection_enabled) return;
    rate_counters.odom_count++;
}

void DebugManager::recordControlLoop(float loop_time_ms) {
    if (!metrics_collection_enabled) return;
    
    rate_counters.control_count++;
    
    // Update timing metrics
    float current_avg = metrics.average_loop_time_ms.load();
    float new_avg = (current_avg * 0.9f) + (loop_time_ms * 0.1f);  // Exponential moving average
    metrics.average_loop_time_ms.store(new_avg);
    
    float current_max = metrics.max_loop_time_ms.load();
    if (loop_time_ms > current_max) {
        metrics.max_loop_time_ms.store(loop_time_ms);
    }
    
    // Check for loop overruns (assuming 200ms target)
    if (loop_time_ms > 250.0f) {
        metrics.loop_overruns.fetch_add(1);
    }
}

void DebugManager::recordSPIError() {
    if (!metrics_collection_enabled) return;
    metrics.spi_errors.fetch_add(1);
}

void DebugManager::recordConnectionDrop() {
    if (!metrics_collection_enabled) return;
    metrics.connection_drops.fetch_add(1);
}

void DebugManager::recordCommandQueueOverflow() {
    if (!metrics_collection_enabled) return;
    metrics.command_queue_overflows.fetch_add(1);
}

void DebugManager::recordSensorReadFailure() {
    if (!metrics_collection_enabled) return;
    metrics.sensor_read_failures.fetch_add(1);
}

void DebugManager::recordOdometryCalculationTime(float time_ms) {
    if (!metrics_collection_enabled) return;
    
    // Update average calculation time using exponential moving average
    float current_avg = metrics.odometry_avg_calc_time_ms.load();
    float new_avg = (current_avg * 0.9f) + (time_ms * 0.1f);
    metrics.odometry_avg_calc_time_ms.store(new_avg);
    
    // Update max calculation time
    float current_max = metrics.odometry_max_calc_time_ms.load();
    if (time_ms > current_max) {
        metrics.odometry_max_calc_time_ms.store(time_ms);
    }
}

void DebugManager::recordOdometryStaleData() {
    if (!metrics_collection_enabled) return;
    metrics.odometry_stale_data_events.fetch_add(1);
}

void DebugManager::recordOdometryInvalidData() {
    if (!metrics_collection_enabled) return;
    metrics.odometry_invalid_data_events.fetch_add(1);
}

void DebugManager::recordOdometryError() {
    if (!metrics_collection_enabled) return;
    metrics.odometry_calculation_errors.fetch_add(1);
}

void DebugManager::calculateRates() {
    uint32_t now = millis();
    uint32_t dt = now - rate_counters.last_rate_calc_time;
    
    if (dt >= 1000) {  // Calculate rates every second
        float dt_sec = dt / 1000.0f;
        
        metrics.twist_callbacks_per_sec.store((uint32_t)(rate_counters.twist_count / dt_sec));
        metrics.odom_publishes_per_sec.store((uint32_t)(rate_counters.odom_count / dt_sec));
        metrics.control_loop_frequency.store((uint32_t)(rate_counters.control_count / dt_sec));
        
        // Reset counters
        rate_counters.twist_count = 0;
        rate_counters.odom_count = 0;
        rate_counters.control_count = 0;
        rate_counters.last_rate_calc_time = now;
    }
}

void DebugManager::updateSystemHealth() {
    // Update uptime
    uint32_t uptime_ms = millis() - system_start_time;
    metrics.uptime_seconds.store(uptime_ms / 1000);
    
    // Check ROS connection health
    health.ros_connection_healthy = (metrics.connection_drops.load() == 0) || 
                                   (millis() - rate_counters.last_rate_calc_time < 5000);
    
    // Check motor health (based on error rates)
    uint32_t spi_errors = metrics.spi_errors.load();
    health.motors_healthy = (spi_errors < 10) || (spi_errors / (uptime_ms / 1000.0f) < 0.1f);
    
    // Check sensor health
    uint32_t sensor_failures = metrics.sensor_read_failures.load();
    health.sensors_healthy = (sensor_failures < 5) || (sensor_failures / (uptime_ms / 1000.0f) < 0.05f);
    
    // Check memory health
    size_t free_mem = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    metrics.free_memory_bytes.store(free_mem);
    health.memory_healthy = free_mem > 50000;  // At least 50KB free
    
    // Check timing health
    float avg_loop_time = metrics.average_loop_time_ms.load();
    uint32_t overruns = metrics.loop_overruns.load();
    health.timing_healthy = (avg_loop_time < 220.0f) && (overruns < 10);
    
    // Calculate overall health score
    health.overall_health_score = calculateOverallHealthScore();
}

float DebugManager::calculateOverallHealthScore() {
    float score = 0.0f;
    int components = 0;
    
    if (health.ros_connection_healthy) score += 0.3f;
    components++;
    
    if (health.motors_healthy) score += 0.25f;
    components++;
    
    if (health.sensors_healthy) score += 0.2f;
    components++;
    
    if (health.memory_healthy) score += 0.15f;
    components++;
    
    if (health.timing_healthy) score += 0.1f;
    components++;
    
    return score;  // Already weighted, no need to divide by components
}

void DebugManager::publishMetrics() {
    if (!ros_logging_enabled || metrics_publisher == NULL) return;
    
    if (debug_mutex == NULL || xSemaphoreTake(debug_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
    
    if (metrics_msg.data.data != NULL) {
        metrics_msg.data.size = 20;  // Number of metrics we're publishing (increased for odometry)
        
        // Performance metrics
        metrics_msg.data.data[0] = (float)metrics.twist_callbacks_per_sec.load();
        metrics_msg.data.data[1] = (float)metrics.odom_publishes_per_sec.load();
        metrics_msg.data.data[2] = (float)metrics.control_loop_frequency.load();
        metrics_msg.data.data[3] = metrics.average_loop_time_ms.load();
        metrics_msg.data.data[4] = metrics.max_loop_time_ms.load();
        
        // Error counters
        metrics_msg.data.data[5] = (float)metrics.loop_overruns.load();
        metrics_msg.data.data[6] = (float)metrics.spi_errors.load();
        metrics_msg.data.data[7] = (float)metrics.connection_drops.load();
        metrics_msg.data.data[8] = (float)metrics.command_queue_overflows.load();
        metrics_msg.data.data[9] = (float)metrics.sensor_read_failures.load();
        
        // System health
        metrics_msg.data.data[10] = metrics.cpu_usage_percent.load();
        metrics_msg.data.data[11] = (float)metrics.free_memory_bytes.load();
        metrics_msg.data.data[12] = (float)metrics.uptime_seconds.load();
        metrics_msg.data.data[13] = health.overall_health_score;
        metrics_msg.data.data[14] = (float)(health.ros_connection_healthy ? 1 : 0);
        
        // Odometry-specific metrics
        metrics_msg.data.data[15] = (float)metrics.odometry_stale_data_events.load();
        metrics_msg.data.data[16] = (float)metrics.odometry_invalid_data_events.load();
        metrics_msg.data.data[17] = (float)metrics.odometry_calculation_errors.load();
        metrics_msg.data.data[18] = metrics.odometry_avg_calc_time_ms.load();
        metrics_msg.data.data[19] = metrics.odometry_max_calc_time_ms.load();
        
        rcl_publish(metrics_publisher, &metrics_msg, NULL);
    }
    
    xSemaphoreGive(debug_mutex);
}

void DebugManager::update() {
    uint32_t now = millis();
    
    // Calculate rates periodically
    calculateRates();
    
    // Update system health
    if (now - last_health_check >= HEALTH_CHECK_INTERVAL_MS) {
        updateSystemHealth();
        last_health_check = now;
    }
    
    // Publish metrics periodically
    if (now - last_metrics_publish >= METRICS_PUBLISH_INTERVAL_MS) {
        publishMetrics();
        last_metrics_publish = now;
        
        // Reset max times after publishing
        metrics.max_loop_time_ms.store(0.0f);
        metrics.odometry_max_calc_time_ms.store(0.0f);
    }
}

const PerformanceMetrics& DebugManager::getMetrics() const {
    return metrics;
}

const SystemHealthIndicators& DebugManager::getHealthIndicators() const {
    return health;
}

void DebugManager::resetMetrics() {
    metrics.reset();
    rate_counters.twist_count = 0;
    rate_counters.odom_count = 0;
    rate_counters.control_count = 0;
    rate_counters.last_rate_calc_time = millis();
}

void DebugManager::publishDebugData() {
    // Force immediate debug data publish
    if (debug_publisher != NULL) {
        log(DebugLevel::INFO, "manual_debug_publish");
    }
}

void DebugManager::publishCurrentMetrics() {
    // Force immediate metrics publish
    publishMetrics();
}

void DebugManager::updateMemoryUsage() {
    size_t free_mem = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    metrics.free_memory_bytes.store(free_mem);
}

void DebugManager::updateCPUUsage() {
    // Simple CPU usage estimation based on loop timing
    // This is a rough approximation for ESP32
    static uint32_t last_idle_time = 0;
    static uint32_t last_check_time = 0;
    
    uint32_t now = millis();
    uint32_t dt = now - last_check_time;
    
    if (dt > 1000) {  // Update every second
        // This is a simplified approach - real CPU usage would require more sophisticated tracking
        float avg_loop_time = metrics.average_loop_time_ms.load();
        float estimated_cpu = (avg_loop_time / 200.0f) * 100.0f;  // Assuming 200ms target loop time
        
        if (estimated_cpu > 100.0f) estimated_cpu = 100.0f;
        if (estimated_cpu < 0.0f) estimated_cpu = 0.0f;
        
        metrics.cpu_usage_percent.store(estimated_cpu);
        last_check_time = now;
    }
}

bool DebugManager::isSystemHealthy() const {
    return health.overall_health_score > 0.7f;  // 70% threshold
}

float DebugManager::getHealthScore() const {
    return health.overall_health_score;
}