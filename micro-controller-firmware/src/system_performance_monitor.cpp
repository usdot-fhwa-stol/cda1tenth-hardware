#include "system_performance_monitor.h"
#include "connection_manager.h"
#include "car.h"
#include "sensor_cache.h"
#include "debug_manager.h"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <algorithm>

SystemPerformanceMonitor::SystemPerformanceMonitor() 
    : connection_manager_(nullptr)
    , car_(nullptr)
    , sensor_cache_(nullptr)
    , last_update_time_(0)
    , last_health_check_(0)
    , last_alert_time_(0)
    , health_alerting_enabled_(true)
{
    state_mutex_ = xSemaphoreCreateMutex();
    
    // Initialize rate counters
    rate_counters_.twist_count = 0;
    rate_counters_.odom_count = 0;
    rate_counters_.debug_count = 0;
    rate_counters_.last_rate_calc_time = 0;
    
    // Initialize CPU tracker
    cpu_tracker_.last_idle_time = 0;
    cpu_tracker_.last_total_time = 0;
    cpu_tracker_.last_measurement_time = 0;
    
    // Initialize memory tracker
    memory_tracker_.baseline_free_memory = 0;
    memory_tracker_.min_free_memory = SIZE_MAX;
    memory_tracker_.allocation_failures = 0;
    
    // Set system start time
    current_state_.system_start_time = millis();
    current_state_.last_update_time = current_state_.system_start_time;
}

SystemPerformanceMonitor::~SystemPerformanceMonitor() {
    shutdown();
    if (state_mutex_ != NULL) {
        vSemaphoreDelete(state_mutex_);
    }
}

bool SystemPerformanceMonitor::initialize(ConnectionManager* conn_mgr, Car* car, SensorCache* sensor_cache) {
    if (conn_mgr == nullptr || car == nullptr || sensor_cache == nullptr) {
        return false;
    }
    
    connection_manager_ = conn_mgr;
    car_ = car;
    sensor_cache_ = sensor_cache;
    
    // Initialize baseline memory measurement
    memory_tracker_.baseline_free_memory = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    current_state_.total_memory_bytes = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    
    // Initialize timing
    last_update_time_ = millis();
    last_health_check_ = last_update_time_;
    rate_counters_.last_rate_calc_time = last_update_time_;
    
    return true;
}

void SystemPerformanceMonitor::shutdown() {
    // Clean shutdown - reset pointers
    connection_manager_ = nullptr;
    car_ = nullptr;
    sensor_cache_ = nullptr;
}

void SystemPerformanceMonitor::setHealthThresholds(const SystemHealthThresholds& thresholds) {
    if (state_mutex_ != NULL && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        thresholds_ = thresholds;
        xSemaphoreGive(state_mutex_);
    }
}

SystemHealthThresholds SystemPerformanceMonitor::getHealthThresholds() const {
    return thresholds_;
}

void SystemPerformanceMonitor::update() {
    uint32_t now = millis();
    
    // Update system metrics
    if (now - last_update_time_ >= UPDATE_INTERVAL_MS) {
        updateSystemMetrics();
        last_update_time_ = now;
    }
    
    // Update health indicators
    if (now - last_health_check_ >= HEALTH_CHECK_INTERVAL_MS) {
        updateHealthIndicators();
        last_health_check_ = now;
    }
    
    // Calculate communication rates
    calculateRates();
}

SystemState SystemPerformanceMonitor::getSystemState() const {
    if (state_mutex_ != NULL && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        SystemState state_copy = current_state_;
        xSemaphoreGive(state_mutex_);
        return state_copy;
    }
    return current_state_;  // Return without mutex if unavailable
}

bool SystemPerformanceMonitor::isSystemHealthy() const {
    return current_state_.overall_health_score >= thresholds_.min_health_score;
}

float SystemPerformanceMonitor::getHealthScore() const {
    return current_state_.overall_health_score;
}

void SystemPerformanceMonitor::recordTwistCallback() {
    rate_counters_.twist_count++;
}

void SystemPerformanceMonitor::recordOdomPublish() {
    rate_counters_.odom_count++;
}

void SystemPerformanceMonitor::recordDebugMessage() {
    rate_counters_.debug_count++;
}

void SystemPerformanceMonitor::recordLoopTiming(float loop_time_ms) {
    if (state_mutex_ != NULL && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1)) == pdTRUE) {
        // Update average using exponential moving average
        current_state_.average_loop_time_ms = (current_state_.average_loop_time_ms * 0.9f) + (loop_time_ms * 0.1f);
        
        // Update maximum
        if (loop_time_ms > current_state_.max_loop_time_ms) {
            current_state_.max_loop_time_ms = loop_time_ms;
        }
        
        // Count overruns
        if (loop_time_ms > thresholds_.max_loop_time_ms) {
            current_state_.loop_overruns++;
        }
        
        xSemaphoreGive(state_mutex_);
    }
}

void SystemPerformanceMonitor::recordSPIError() {
    current_state_.spi_errors++;
}

void SystemPerformanceMonitor::recordConnectionDrop() {
    current_state_.connection_drops++;
}

void SystemPerformanceMonitor::recordSensorFailure() {
    current_state_.sensor_failures++;
}

void SystemPerformanceMonitor::recordMemoryAllocationFailure() {
    current_state_.memory_allocation_failures++;
    memory_tracker_.allocation_failures++;
}

void SystemPerformanceMonitor::setCarInitialized(bool initialized) {
    current_state_.car_initialized = initialized;
}

void SystemPerformanceMonitor::setROSInitialized(bool initialized) {
    current_state_.ros_initialized = initialized;
}

void SystemPerformanceMonitor::setSensorsInitialized(bool initialized) {
    current_state_.sensors_initialized = initialized;
}

void SystemPerformanceMonitor::setSystemComponentsInitialized(bool initialized) {
    current_state_.system_components_initialized = initialized;
}

void SystemPerformanceMonitor::updateSystemMetrics() {
    if (state_mutex_ != NULL && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint32_t now = millis();
        current_state_.uptime_ms = now - current_state_.system_start_time;
        current_state_.last_update_time = now;
        
        // Update individual metric categories
        updateCPUUsage();
        updateMemoryUsage();
        updateTimingMetrics();
        updateConnectionMetrics();
        updateHardwareHealth();
        
        xSemaphoreGive(state_mutex_);
    }
}

void SystemPerformanceMonitor::updateHealthIndicators() {
    if (state_mutex_ != NULL && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Update health flags
        current_state_.memory_healthy = isMemoryHealthy();
        current_state_.timing_healthy = isTimingHealthy();
        current_state_.motors_healthy = areMotorsHealthy();
        current_state_.sensors_healthy = areSensorsHealthy();
        current_state_.spi_healthy = isSPIHealthy();
        
        // Calculate overall health score
        current_state_.overall_health_score = calculateHealthScore();
        
        // Check thresholds and trigger alerts if needed
        if (health_alerting_enabled_) {
            checkHealthThresholds();
        }
        
        xSemaphoreGive(state_mutex_);
    }
}

void SystemPerformanceMonitor::calculateRates() {
    uint32_t now = millis();
    uint32_t dt = now - rate_counters_.last_rate_calc_time;
    
    if (dt >= 1000) {  // Calculate rates every second
        float dt_sec = dt / 1000.0f;
        
        if (state_mutex_ != NULL && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
            current_state_.twist_callbacks_per_sec = (uint32_t)(rate_counters_.twist_count / dt_sec);
            current_state_.odom_publishes_per_sec = (uint32_t)(rate_counters_.odom_count / dt_sec);
            current_state_.debug_messages_per_sec = (uint32_t)(rate_counters_.debug_count / dt_sec);
            
            xSemaphoreGive(state_mutex_);
        }
        
        // Reset counters
        rate_counters_.twist_count = 0;
        rate_counters_.odom_count = 0;
        rate_counters_.debug_count = 0;
        rate_counters_.last_rate_calc_time = now;
    }
}

void SystemPerformanceMonitor::updateCPUUsage() {
    // Simple CPU usage estimation based on loop timing and system load
    // This is an approximation since ESP32 doesn't provide direct CPU usage APIs
    
    uint32_t now = millis();
    if (now - cpu_tracker_.last_measurement_time >= 1000) {  // Update every second
        
        // Estimate CPU usage based on average loop time
        float target_loop_time = 200.0f;  // Target 200ms loop time
        float actual_loop_time = current_state_.average_loop_time_ms;
        
        if (actual_loop_time > 0) {
            float cpu_estimate = (actual_loop_time / target_loop_time) * 100.0f;
            
            // Clamp to reasonable bounds
            cpu_estimate = std::max(0.0f, std::min(100.0f, cpu_estimate));
            
            // Apply smoothing
            current_state_.cpu_usage_percent = (current_state_.cpu_usage_percent * 0.8f) + (cpu_estimate * 0.2f);
        }
        
        cpu_tracker_.last_measurement_time = now;
    }
}

void SystemPerformanceMonitor::updateMemoryUsage() {
    size_t free_memory = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    current_state_.free_memory_bytes = free_memory;
    
    // Track minimum free memory
    if (free_memory < memory_tracker_.min_free_memory) {
        memory_tracker_.min_free_memory = free_memory;
    }
}

void SystemPerformanceMonitor::updateTimingMetrics() {
    // Reset max loop time periodically (every update cycle)
    static uint32_t last_reset = 0;
    uint32_t now = millis();
    
    if (now - last_reset >= UPDATE_INTERVAL_MS) {
        current_state_.max_loop_time_ms = 0.0f;  // Reset for next measurement period
        last_reset = now;
    }
}

void SystemPerformanceMonitor::updateConnectionMetrics() {
    if (connection_manager_ != nullptr) {
        // Map connection manager state to our enum
        ConnectionManager::State conn_state = connection_manager_->getState();
        switch (conn_state) {
            case ConnectionManager::WAITING:
                current_state_.connection_state = SystemState::DISCONNECTED;
                break;
            case ConnectionManager::AVAILABLE:
                current_state_.connection_state = SystemState::CONNECTING;
                break;
            case ConnectionManager::CONNECTED:
                current_state_.connection_state = connection_manager_->isHealthy() ? 
                    SystemState::CONNECTED : SystemState::DEGRADED;
                break;
            case ConnectionManager::DISCONNECTED:
                current_state_.connection_state = SystemState::DISCONNECTED;
                break;
            case ConnectionManager::ERROR:
                current_state_.connection_state = SystemState::DISCONNECTED;
                break;
        }
    }
}

void SystemPerformanceMonitor::updateHardwareHealth() {
    // Update hardware health based on error rates and system state
    
    // SPI health based on error rate
    uint32_t uptime_minutes = current_state_.uptime_ms / 60000;
    if (uptime_minutes > 0) {
        uint32_t spi_errors_per_minute = current_state_.spi_errors / uptime_minutes;
        current_state_.spi_healthy = spi_errors_per_minute <= thresholds_.max_spi_errors_per_minute;
    } else {
        current_state_.spi_healthy = current_state_.spi_errors == 0;
    }
}

float SystemPerformanceMonitor::calculateHealthScore() {
    float score = 0.0f;
    
    // Connection health (25% weight)
    if (current_state_.connection_state == SystemState::CONNECTED) {
        score += 0.25f;
    } else if (current_state_.connection_state == SystemState::DEGRADED) {
        score += 0.15f;
    }
    
    // Memory health (20% weight)
    if (current_state_.memory_healthy) {
        score += 0.20f;
    } else {
        // Partial credit based on available memory
        float memory_mb = current_state_.free_memory_bytes / (1024.0f * 1024.0f);
        float memory_ratio = memory_mb / thresholds_.min_memory_mb;
        score += 0.20f * std::min(1.0f, memory_ratio);
    }
    
    // Timing health (20% weight)
    if (current_state_.timing_healthy) {
        score += 0.20f;
    } else {
        // Partial credit based on loop time
        float timing_ratio = thresholds_.max_loop_time_ms / std::max(1.0f, current_state_.average_loop_time_ms);
        score += 0.20f * std::min(1.0f, timing_ratio);
    }
    
    // Motor health (15% weight)
    if (current_state_.motors_healthy) {
        score += 0.15f;
    }
    
    // Sensor health (10% weight)
    if (current_state_.sensors_healthy) {
        score += 0.10f;
    }
    
    // SPI health (10% weight)
    if (current_state_.spi_healthy) {
        score += 0.10f;
    }
    
    return std::min(1.0f, score);
}

bool SystemPerformanceMonitor::checkHealthThresholds() {
    uint32_t now = millis();
    bool alert_triggered = false;
    
    // Avoid alert spam
    if (now - last_alert_time_ < ALERT_COOLDOWN_MS) {
        return false;
    }
    
    // Check memory threshold
    float memory_mb = current_state_.free_memory_bytes / (1024.0f * 1024.0f);
    if (memory_mb < thresholds_.min_memory_mb) {
        triggerHealthAlert("LOW_MEMORY", memory_mb, thresholds_.min_memory_mb);
        alert_triggered = true;
    }
    
    // Check CPU usage threshold
    if (current_state_.cpu_usage_percent > thresholds_.max_cpu_usage_percent) {
        triggerHealthAlert("HIGH_CPU", current_state_.cpu_usage_percent, thresholds_.max_cpu_usage_percent);
        alert_triggered = true;
    }
    
    // Check loop time threshold
    if (current_state_.average_loop_time_ms > thresholds_.max_loop_time_ms) {
        triggerHealthAlert("SLOW_LOOP", current_state_.average_loop_time_ms, thresholds_.max_loop_time_ms);
        alert_triggered = true;
    }
    
    // Check overall health score
    if (current_state_.overall_health_score < thresholds_.min_health_score) {
        triggerHealthAlert("LOW_HEALTH", current_state_.overall_health_score, thresholds_.min_health_score);
        alert_triggered = true;
    }
    
    if (alert_triggered) {
        last_alert_time_ = now;
    }
    
    return alert_triggered;
}

void SystemPerformanceMonitor::triggerHealthAlert(const char* alert_type, float value, float threshold) {
    // Log health alert through debug manager
    DEBUG_WARN("HEALTH_ALERT");
    
    // Could also publish specific alert messages via ROS if needed
}

bool SystemPerformanceMonitor::isMemoryHealthy() const {
    float memory_mb = current_state_.free_memory_bytes / (1024.0f * 1024.0f);
    return memory_mb >= thresholds_.min_memory_mb;
}

bool SystemPerformanceMonitor::isTimingHealthy() const {
    return current_state_.average_loop_time_ms <= thresholds_.max_loop_time_ms &&
           current_state_.loop_overruns < 10;  // Allow some overruns
}

bool SystemPerformanceMonitor::areMotorsHealthy() const {
    if (car_ == nullptr) return false;
    
    // Check if car is initialized and SPI is healthy
    return current_state_.car_initialized && current_state_.spi_healthy;
}

bool SystemPerformanceMonitor::areSensorsHealthy() const {
    if (sensor_cache_ == nullptr) return false;
    
    // Check sensor failure rate
    uint32_t uptime_minutes = current_state_.uptime_ms / 60000;
    if (uptime_minutes > 0) {
        uint32_t failures_per_minute = current_state_.sensor_failures / uptime_minutes;
        return failures_per_minute <= 2;  // Allow up to 2 failures per minute
    }
    
    return current_state_.sensor_failures == 0;
}

bool SystemPerformanceMonitor::isSPIHealthy() const {
    return current_state_.spi_healthy;
}

uint32_t SystemPerformanceMonitor::getUptime() const {
    return current_state_.uptime_ms;
}

float SystemPerformanceMonitor::getCPUUsage() const {
    return current_state_.cpu_usage_percent;
}

size_t SystemPerformanceMonitor::getFreeMemory() const {
    return current_state_.free_memory_bytes;
}

float SystemPerformanceMonitor::getAverageLoopTime() const {
    return current_state_.average_loop_time_ms;
}

uint32_t SystemPerformanceMonitor::getErrorCount(const char* error_type) const {
    if (strcmp(error_type, "spi") == 0) {
        return current_state_.spi_errors;
    } else if (strcmp(error_type, "connection") == 0) {
        return current_state_.connection_drops;
    } else if (strcmp(error_type, "sensor") == 0) {
        return current_state_.sensor_failures;
    } else if (strcmp(error_type, "memory") == 0) {
        return current_state_.memory_allocation_failures;
    }
    return 0;
}

void SystemPerformanceMonitor::enableHealthAlerting(bool enable) {
    health_alerting_enabled_ = enable;
}

bool SystemPerformanceMonitor::isHealthAlertingEnabled() const {
    return health_alerting_enabled_;
}