#ifndef TASK_PERFORMANCE_H
#define TASK_PERFORMANCE_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// COMMON TASK PERFORMANCE METRICS
// =============================================================================

struct TaskPerformanceMetrics {
    uint32_t max_execution_time_us;
    uint32_t avg_execution_time_us;
    uint32_t min_execution_time_us;
    uint32_t execution_count;
    uint32_t missed_deadlines;
    uint32_t last_update_timestamp;
    
    // Task-specific error counters
    uint32_t task_specific_errors;
    
    // Initialize with default values
    void init() {
        max_execution_time_us = 0;
        avg_execution_time_us = 0;
        min_execution_time_us = UINT32_MAX;
        execution_count = 0;
        missed_deadlines = 0;
        task_specific_errors = 0;
        last_update_timestamp = 0;
    }
    
    // Common performance update logic
    void update(uint32_t executionTimeUs) {
        execution_count++;
        last_update_timestamp = micros();
        
        // Update min/max
        if (executionTimeUs > max_execution_time_us) {
            max_execution_time_us = executionTimeUs;
        }
        if (executionTimeUs < min_execution_time_us) {
            min_execution_time_us = executionTimeUs;
        }
        
        // Update average (simple moving average)
        avg_execution_time_us = 
            (avg_execution_time_us * (execution_count - 1) + executionTimeUs) / 
            execution_count;
    }
    
    // Reset all metrics
    void reset() {
        init();
    }
};

// =============================================================================
// TASK-SPECIFIC PERFORMANCE METRICS
// =============================================================================

struct MotorControlPerformanceMetrics : public TaskPerformanceMetrics {
    uint32_t spi_errors;
    uint32_t queue_overruns;
    
    void init() {
        TaskPerformanceMetrics::init();
        spi_errors = 0;
        queue_overruns = 0;
    }
    
    void reset() {
        init();
    }
};

struct ROSCommPerformanceMetrics : public TaskPerformanceMetrics {
    uint32_t ros_connection_drops;
    uint32_t ros_message_send_failures;
    uint32_t ros_message_receive_count;
    uint32_t agent_reconnection_attempts;
    
    void init() {
        TaskPerformanceMetrics::init();
        ros_connection_drops = 0;
        ros_message_send_failures = 0;
        ros_message_receive_count = 0;
        agent_reconnection_attempts = 0;
    }
    
    void reset() {
        init();
    }
};

#endif // TASK_PERFORMANCE_H
