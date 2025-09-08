#include <unity.h>
#include <Arduino.h>
#include "ros_comm_task.h"
#include "inter_core_comm.h"

void test_ros_comm_task_creation(void) {
    ROSCommTask task;
    
    TEST_ASSERT_FALSE(task.isRunning());
    TEST_ASSERT_FALSE(task.isAgentConnected());
}

void test_ros_comm_task_initialization(void) {
    ROSCommTask task;
    
    // Initialize inter-core communication for testing
    InterCoreCommunication& comm = InterCoreCommunication::getInstance();
    comm.initialize();
    ThreadSafeCarState& carState = comm.getCarState();
    
    // Test initialization
    TEST_ASSERT_TRUE(task.initialize(&carState, DEFAULT_SYSTEM_CONFIG));
    
    // Cleanup
    task.cleanup();
    comm.cleanup();
}

void test_ros_comm_task_performance_metrics(void) {
    ROSCommTask task;
    
    // Test performance metrics
    ROSCommTask::PerformanceMetrics metrics = task.getPerformanceMetrics();
    TEST_ASSERT_EQUAL(0, metrics.execution_count);
    TEST_ASSERT_EQUAL(0, metrics.max_execution_time_us);
    TEST_ASSERT_EQUAL(0, metrics.avg_execution_time_us);
    TEST_ASSERT_EQUAL(UINT32_MAX, metrics.min_execution_time_us);
    TEST_ASSERT_EQUAL(0, metrics.missed_deadlines);
    TEST_ASSERT_EQUAL(0, metrics.ros_connection_drops);
    TEST_ASSERT_EQUAL(0, metrics.ros_message_send_failures);
    TEST_ASSERT_EQUAL(0, metrics.ros_message_receive_count);
    TEST_ASSERT_EQUAL(0, metrics.agent_reconnection_attempts);
    TEST_ASSERT_EQUAL(0, metrics.last_update_timestamp);
}

void test_ros_comm_task_health_check(void) {
    ROSCommTask task;
    
    // Before initialization
    TEST_ASSERT_FALSE(task.isHealthy());
    
    // Initialize and test health
    InterCoreCommunication& comm = InterCoreCommunication::getInstance();
    comm.initialize();
    ThreadSafeCarState& carState = comm.getCarState();
    
    if (task.initialize(&carState, DEFAULT_SYSTEM_CONFIG)) {
        TEST_ASSERT_TRUE(task.isHealthy());
        task.cleanup();
    }
    
    comm.cleanup();
}

void test_ros_comm_task_error_counting(void) {
    ROSCommTask task;
    
    // Test error counting
    TEST_ASSERT_EQUAL(0, task.getErrorCount());
    
    // Initialize for testing
    InterCoreCommunication& comm = InterCoreCommunication::getInstance();
    comm.initialize();
    ThreadSafeCarState& carState = comm.getCarState();
    
    if (task.initialize(&carState, DEFAULT_SYSTEM_CONFIG)) {
        // Error count should still be 0 after initialization
        TEST_ASSERT_EQUAL(0, task.getErrorCount());
        task.cleanup();
    }
    
    comm.cleanup();
}

void test_ros_comm_task_agent_connection(void) {
    ROSCommTask task;
    
    // Before initialization
    TEST_ASSERT_FALSE(task.isAgentConnected());
    
    // Initialize and test agent connection
    InterCoreCommunication& comm = InterCoreCommunication::getInstance();
    comm.initialize();
    ThreadSafeCarState& carState = comm.getCarState();
    
    if (task.initialize(&carState, DEFAULT_SYSTEM_CONFIG)) {
        // Should not be connected without ROS agent
        TEST_ASSERT_FALSE(task.isAgentConnected());
        task.cleanup();
    }
    
    comm.cleanup();
}

void test_ros_comm_task_force_reconnection(void) {
    ROSCommTask task;
    
    // Initialize for testing
    InterCoreCommunication& comm = InterCoreCommunication::getInstance();
    comm.initialize();
    ThreadSafeCarState& carState = comm.getCarState();
    
    if (task.initialize(&carState, DEFAULT_SYSTEM_CONFIG)) {
        // Test force reconnection
        task.forceReconnection();
        TEST_ASSERT_FALSE(task.isAgentConnected());
        task.cleanup();
    }
    
    comm.cleanup();
}

void test_ros_comm_task_performance_reset(void) {
    ROSCommTask task;
    
    // Test performance metrics reset
    task.resetPerformanceMetrics();
    
    ROSCommTask::PerformanceMetrics metrics = task.getPerformanceMetrics();
    TEST_ASSERT_EQUAL(0, metrics.execution_count);
    TEST_ASSERT_EQUAL(0, metrics.max_execution_time_us);
    TEST_ASSERT_EQUAL(0, metrics.avg_execution_time_us);
    TEST_ASSERT_EQUAL(UINT32_MAX, metrics.min_execution_time_us);
    TEST_ASSERT_EQUAL(0, metrics.missed_deadlines);
    TEST_ASSERT_EQUAL(0, metrics.ros_connection_drops);
    TEST_ASSERT_EQUAL(0, metrics.ros_message_send_failures);
    TEST_ASSERT_EQUAL(0, metrics.ros_message_receive_count);
    TEST_ASSERT_EQUAL(0, metrics.agent_reconnection_attempts);
}

void test_ros_comm_task_cleanup(void) {
    ROSCommTask task;
    
    // Initialize
    InterCoreCommunication& comm = InterCoreCommunication::getInstance();
    comm.initialize();
    ThreadSafeCarState& carState = comm.getCarState();
    
    if (task.initialize(&carState, DEFAULT_SYSTEM_CONFIG)) {
        TEST_ASSERT_TRUE(task.isHealthy());
        
        // Cleanup
        task.cleanup();
        TEST_ASSERT_FALSE(task.isHealthy());
    }
    
    comm.cleanup();
}

void test_ros_comm_task_multiple_initialization(void) {
    ROSCommTask task;
    
    // Initialize inter-core communication
    InterCoreCommunication& comm = InterCoreCommunication::getInstance();
    comm.initialize();
    ThreadSafeCarState& carState = comm.getCarState();
    
    // First initialization
    bool firstInit = task.initialize(&carState, DEFAULT_SYSTEM_CONFIG);
    
    // Second initialization should return true (already initialized)
    bool secondInit = task.initialize(&carState, DEFAULT_SYSTEM_CONFIG);
    
    if (firstInit) {
        TEST_ASSERT_TRUE(secondInit);
        task.cleanup();
    }
    
    comm.cleanup();
}

void test_ros_comm_task_destructor(void) {
    // Test that destructor cleans up properly
    {
        ROSCommTask task;
        
        InterCoreCommunication& comm = InterCoreCommunication::getInstance();
        comm.initialize();
        ThreadSafeCarState& carState = comm.getCarState();
        
        if (task.initialize(&carState, DEFAULT_SYSTEM_CONFIG)) {
            TEST_ASSERT_TRUE(task.isHealthy());
        }
        // Destructor should clean up automatically
        comm.cleanup();
    }
    
    // Should not crash
    TEST_ASSERT_TRUE(true);
}

int runUnityTests_ros_comm_task(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_ros_comm_task_creation);
    RUN_TEST(test_ros_comm_task_initialization);
    RUN_TEST(test_ros_comm_task_performance_metrics);
    RUN_TEST(test_ros_comm_task_health_check);
    RUN_TEST(test_ros_comm_task_error_counting);
    RUN_TEST(test_ros_comm_task_agent_connection);
    RUN_TEST(test_ros_comm_task_force_reconnection);
    RUN_TEST(test_ros_comm_task_performance_reset);
    RUN_TEST(test_ros_comm_task_cleanup);
    RUN_TEST(test_ros_comm_task_multiple_initialization);
    RUN_TEST(test_ros_comm_task_destructor);
    
    return UNITY_END();
}
