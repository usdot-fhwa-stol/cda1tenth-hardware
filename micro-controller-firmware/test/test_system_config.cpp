#include <unity.h>
#include <Arduino.h>
#include "system_config.h"

void test_default_system_config(void) {
    SystemConfig_t config = DEFAULT_SYSTEM_CONFIG;
    
    // Test default values
    TEST_ASSERT_EQUAL(MOTOR_CONTROL_TASK_PRIORITY, config.motor_task_priority);
    TEST_ASSERT_EQUAL(ROS_COMM_TASK_PRIORITY, config.ros_task_priority);
    TEST_ASSERT_EQUAL(MOTOR_TASK_STACK_SIZE, config.motor_task_stack_size);
    TEST_ASSERT_EQUAL(MOTOR_TASK_CORE_ID, config.motor_task_core_id);
    TEST_ASSERT_EQUAL(ROS_TASK_CORE_ID, config.ros_task_core_id);
    TEST_ASSERT_EQUAL(MOTOR_CONTROL_FREQUENCY_HZ, config.motor_control_frequency_hz);
    TEST_ASSERT_EQUAL(ROS_SPIN_TIMEOUT_MS, config.ros_spin_timeout_ms);
    TEST_ASSERT_EQUAL(COMMAND_QUEUE_SIZE, config.command_queue_size);
    TEST_ASSERT_EQUAL(STATUS_QUEUE_SIZE, config.status_queue_size);
    TEST_ASSERT_EQUAL(SERIAL_BAUD_RATE, config.serial_baud_rate);
    TEST_ASSERT_EQUAL(COMMAND_TIMEOUT_MS, config.command_timeout_ms);
    TEST_ASSERT_EQUAL(WATCHDOG_TIMEOUT_MS, config.watchdog_timeout_ms);
    TEST_ASSERT_EQUAL(AGENT_PING_INTERVAL_MS, config.agent_ping_interval_ms);
    TEST_ASSERT_EQUAL(AGENT_PING_TIMEOUT_MS, config.agent_ping_timeout_ms);
}

void test_motor_command_init(void) {
    MotorCommand_t cmd;
    motor_command_init(&cmd);
    
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.steering_angle);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.drive_speed);
    TEST_ASSERT_EQUAL_FLOAT(0.185f, cmd.wheelbase);
    TEST_ASSERT_EQUAL_FLOAT(0.15f, cmd.track_width);
    TEST_ASSERT_EQUAL(0, cmd.timestamp);
    TEST_ASSERT_FALSE(cmd.emergency_stop);
    TEST_ASSERT_EQUAL(0, cmd.sequence_id);
}

void test_motor_status_init(void) {
    MotorStatus_t status;
    motor_status_init(&status);
    
    TEST_ASSERT_EQUAL_FLOAT(0.0f, status.right_motor_rpm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, status.left_motor_rpm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, status.steering_angle);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, status.steering_position);
    TEST_ASSERT_EQUAL(0, status.timestamp);
    TEST_ASSERT_EQUAL(0, status.error_flags);
    TEST_ASSERT_EQUAL(0, status.sequence_id);
    TEST_ASSERT_FALSE(status.motor_control_active);
}

void test_performance_metrics_init(void) {
    PerformanceMetrics_t metrics;
    performance_metrics_init(&metrics);
    
    TEST_ASSERT_EQUAL(0, metrics.motor_task_max_execution_us);
    TEST_ASSERT_EQUAL(0, metrics.motor_task_avg_execution_us);
    TEST_ASSERT_EQUAL(UINT32_MAX, metrics.motor_task_min_execution_us);
    TEST_ASSERT_EQUAL(0, metrics.ros_task_max_execution_us);
    TEST_ASSERT_EQUAL(0, metrics.ros_task_avg_execution_us);
    TEST_ASSERT_EQUAL(UINT32_MAX, metrics.ros_task_min_execution_us);
    TEST_ASSERT_EQUAL(0, metrics.motor_task_execution_count);
    TEST_ASSERT_EQUAL(0, metrics.ros_task_execution_count);
    TEST_ASSERT_EQUAL(0, metrics.command_queue_overruns);
    TEST_ASSERT_EQUAL(0, metrics.status_queue_overruns);
    TEST_ASSERT_EQUAL(0, metrics.command_queue_max_depth);
    TEST_ASSERT_EQUAL(0, metrics.status_queue_max_depth);
    TEST_ASSERT_EQUAL(0, metrics.spi_communication_errors);
    TEST_ASSERT_EQUAL(0, metrics.ros_connection_drops);
    TEST_ASSERT_EQUAL(0, metrics.ros_message_send_failures);
    TEST_ASSERT_EQUAL(0, metrics.ros_message_receive_count);
    TEST_ASSERT_EQUAL(0, metrics.mutex_timeout_count);
    TEST_ASSERT_EQUAL(0, metrics.watchdog_timeout_count);
    TEST_ASSERT_EQUAL(0, metrics.memory_allocation_failures);
    TEST_ASSERT_EQUAL(0, metrics.core0_cpu_utilization_percent_x100);
    TEST_ASSERT_EQUAL(0, metrics.core1_cpu_utilization_percent_x100);
    TEST_ASSERT_EQUAL(0, metrics.system_uptime_seconds);
    TEST_ASSERT_EQUAL(0, metrics.last_update_timestamp);
}

void test_error_flags(void) {
    // Test error flag definitions
    TEST_ASSERT_EQUAL(1, ERROR_FLAG_SPI_COMM_FAILURE);
    TEST_ASSERT_EQUAL(2, ERROR_FLAG_MOTOR_STALL);
    TEST_ASSERT_EQUAL(4, ERROR_FLAG_COMMAND_TIMEOUT);
    TEST_ASSERT_EQUAL(8, ERROR_FLAG_QUEUE_OVERFLOW);
    TEST_ASSERT_EQUAL(16, ERROR_FLAG_MUTEX_TIMEOUT);
    TEST_ASSERT_EQUAL(32, ERROR_FLAG_WATCHDOG_TIMEOUT);
    TEST_ASSERT_EQUAL(64, ERROR_FLAG_MEMORY_ALLOCATION);
    TEST_ASSERT_EQUAL(128, ERROR_FLAG_DRIVER_FAULT);
}

void test_configuration_constants(void) {
    // Test configuration constants
    TEST_ASSERT_EQUAL(24, MOTOR_CONTROL_TASK_PRIORITY);
    TEST_ASSERT_EQUAL(10, ROS_COMM_TASK_PRIORITY);
    TEST_ASSERT_EQUAL(4096, MOTOR_TASK_STACK_SIZE);
    TEST_ASSERT_EQUAL(0, MOTOR_TASK_CORE_ID);
    TEST_ASSERT_EQUAL(1, ROS_TASK_CORE_ID);
    TEST_ASSERT_EQUAL(1000, MOTOR_CONTROL_FREQUENCY_HZ);
    TEST_ASSERT_EQUAL(1, MOTOR_CONTROL_PERIOD_MS);
    TEST_ASSERT_EQUAL(10, ROS_SPIN_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(4096, MOTOR_CONTROL_TASK_STACK_SIZE);
    TEST_ASSERT_EQUAL(8192, ROS_COMM_TASK_STACK_SIZE);
    TEST_ASSERT_EQUAL(8, COMMAND_QUEUE_SIZE);
    TEST_ASSERT_EQUAL(8, STATUS_QUEUE_SIZE);
    TEST_ASSERT_EQUAL(921600, SERIAL_BAUD_RATE);
    TEST_ASSERT_EQUAL(500, COMMAND_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(1000, WATCHDOG_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(200, AGENT_PING_INTERVAL_MS);
    TEST_ASSERT_EQUAL(100, AGENT_PING_TIMEOUT_MS);
}

void test_null_pointer_handling(void) {
    // Test null pointer handling in init functions
    motor_command_init(nullptr);
    motor_status_init(nullptr);
    performance_metrics_init(nullptr);
    
    // Should not crash
    TEST_ASSERT_TRUE(true);
}

int runUnityTests_system_config(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_default_system_config);
    RUN_TEST(test_motor_command_init);
    RUN_TEST(test_motor_status_init);
    RUN_TEST(test_performance_metrics_init);
    RUN_TEST(test_error_flags);
    RUN_TEST(test_configuration_constants);
    RUN_TEST(test_null_pointer_handling);
    
    return UNITY_END();
}
