#include <unity.h>
#include <Arduino.h>
#include "ros_logger.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>

// Mock ROS2 node for testing
rcl_node_t test_node;
rclc_support_t test_support;
rcl_allocator_t test_allocator;

void setUp_ros_logger(void) {
    // Initialize mock ROS2 context
    test_allocator = rcl_get_default_allocator();
    rclc_support_init(&test_support, 0, NULL, &test_allocator);
    rclc_node_init_default(&test_node, "test_node", "", &test_support);
}

void tearDown_ros_logger(void) {
    // Clean up mock ROS2 context
    rcl_ret_t ret = rcl_node_fini(&test_node);
    (void)ret; // Suppress unused variable warning
    rclc_support_fini(&test_support);
}

void test_ros_logger_initialization(void) {
    setUp_ros_logger();
    
    ROSLogger logger;
    TEST_ASSERT_FALSE(logger.isInitialized());
    
    TEST_ASSERT_TRUE(logger.initialize(&test_node, DEFAULT_SYSTEM_CONFIG));
    TEST_ASSERT_TRUE(logger.isInitialized());
    TEST_ASSERT_TRUE(logger.isPublisherReady());
    
    logger.cleanup();
    TEST_ASSERT_FALSE(logger.isInitialized());
    
    tearDown_ros_logger();
}

void test_log_level_strings(void) {
    setUp_ros_logger();
    
    ROSLogger logger;
    logger.initialize(&test_node, DEFAULT_SYSTEM_CONFIG);
    
    TEST_ASSERT_EQUAL_STRING("DEBUG", logger.getLogLevelString(ROSLogger::LOG_DEBUG));
    TEST_ASSERT_EQUAL_STRING("INFO", logger.getLogLevelString(ROSLogger::LOG_INFO));
    TEST_ASSERT_EQUAL_STRING("WARN", logger.getLogLevelString(ROSLogger::LOG_WARN));
    TEST_ASSERT_EQUAL_STRING("ERROR", logger.getLogLevelString(ROSLogger::LOG_ERROR));
    TEST_ASSERT_EQUAL_STRING("FATAL", logger.getLogLevelString(ROSLogger::LOG_FATAL));
    
    logger.cleanup();
    
    tearDown_ros_logger();
}

void test_basic_logging(void) {
    ROSLogger logger;
    logger.initialize(&test_node, DEFAULT_SYSTEM_CONFIG);
    
    // Test basic logging functions
    logger.debug("TEST", "Debug message");
    logger.info("TEST", "Info message");
    logger.warn("TEST", "Warning message");
    logger.error("TEST", "Error message");
    logger.fatal("TEST", "Fatal message");
    
    // Should not crash
    TEST_ASSERT_TRUE(true);
    
    logger.cleanup();
}

void test_formatted_logging(void) {
    ROSLogger logger;
    logger.initialize(&test_node, DEFAULT_SYSTEM_CONFIG);
    
    // Test formatted logging
    logger.debugf("TEST", "Debug: %d", 42);
    logger.infof("TEST", "Info: %.2f", 3.14f);
    logger.warnf("TEST", "Warning: %s", "test");
    logger.errorf("TEST", "Error: %x", 0xDEADBEEF);
    logger.fatalf("TEST", "Fatal: %u", 12345);
    
    // Should not crash
    TEST_ASSERT_TRUE(true);
    
    logger.cleanup();
}

void test_system_status_logging(void) {
    ROSLogger logger;
    logger.initialize(&test_node, DEFAULT_SYSTEM_CONFIG);
    
    // Test system status logging
    logger.logSystemStatus(true, true, true, true, true, true, 0);
    logger.logSystemStatus(false, false, false, false, false, false, 0x12345678);
    
    // Should not crash
    TEST_ASSERT_TRUE(true);
    
    logger.cleanup();
}

void test_task_info_logging(void) {
    ROSLogger logger;
    logger.initialize(&test_node, DEFAULT_SYSTEM_CONFIG);
    
    // Test task info logging
    logger.logTaskInfo("TestTask", 10, 0, 4096);
    logger.logTaskInfo("AnotherTask", 24, 1, 8192);
    
    // Should not crash
    TEST_ASSERT_TRUE(true);
    
    logger.cleanup();
}

void test_performance_metrics_logging(void) {
    ROSLogger logger;
    logger.initialize(&test_node, DEFAULT_SYSTEM_CONFIG);
    
    // Test performance metrics logging
    logger.logPerformanceMetrics("TestTask", 1000, 150, 5);
    logger.logPerformanceMetrics("AnotherTask", 2000, 200, 0);
    
    // Should not crash
    TEST_ASSERT_TRUE(true);
    
    logger.cleanup();
}

void test_health_warning_logging(void) {
    ROSLogger logger;
    logger.initialize(&test_node, DEFAULT_SYSTEM_CONFIG);
    
    // Test health warning logging
    logger.logHealthWarning("System health check failed");
    logger.logHealthWarning("Memory allocation failed");
    
    // Should not crash
    TEST_ASSERT_TRUE(true);
    
    logger.cleanup();
}

void test_initialization_error_logging(void) {
    ROSLogger logger;
    logger.initialize(&test_node, DEFAULT_SYSTEM_CONFIG);
    
    // Test initialization error logging
    logger.logInitializationError("USB", "Failed to initialize USB CDC");
    logger.logInitializationError("Motor", "SPI communication failed");
    
    // Should not crash
    TEST_ASSERT_TRUE(true);
    
    logger.cleanup();
}

void test_global_logger_functions(void) {
    // Test global logger functions
    TEST_ASSERT_TRUE(initializeROSLogger(&test_node, DEFAULT_SYSTEM_CONFIG));
    
    ROSLogger* logger = getROSLogger();
    TEST_ASSERT_NOT_NULL(logger);
    TEST_ASSERT_TRUE(logger->isInitialized());
    
    cleanupROSLogger();
    TEST_ASSERT_FALSE(logger->isInitialized());
}

void test_logger_without_initialization(void) {
    ROSLogger logger;
    
    // Should not crash when logging without initialization
    logger.debug("TEST", "Should not crash");
    logger.infof("TEST", "Format: %d", 42);
    
    TEST_ASSERT_TRUE(true);
}

int runUnityTests_ros_logger(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_ros_logger_initialization);
    RUN_TEST(test_log_level_strings);
    RUN_TEST(test_basic_logging);
    RUN_TEST(test_formatted_logging);
    RUN_TEST(test_system_status_logging);
    RUN_TEST(test_task_info_logging);
    RUN_TEST(test_performance_metrics_logging);
    RUN_TEST(test_health_warning_logging);
    RUN_TEST(test_initialization_error_logging);
    RUN_TEST(test_global_logger_functions);
    RUN_TEST(test_logger_without_initialization);
    
    return UNITY_END();
}
