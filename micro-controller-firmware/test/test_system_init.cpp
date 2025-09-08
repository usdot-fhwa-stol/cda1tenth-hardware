#include <unity.h>
#include <Arduino.h>
#include "system_init.h"

void test_system_initializer_creation(void) {
    SystemInitializer initializer;
    
    TEST_ASSERT_FALSE(initializer.isSystemInitialized());
    TEST_ASSERT_EQUAL(0, initializer.getInitializationErrors());
}

void test_configuration_validation(void) {
    SystemInitializer initializer;
    
    // Valid configuration
    SystemConfig_t validConfig = DEFAULT_SYSTEM_CONFIG;
    TEST_ASSERT_TRUE(initializer.initializeSystem(validConfig));
    TEST_ASSERT_TRUE(initializer.isSystemInitialized());
    
    initializer.shutdownSystem();
}

void test_system_health_check(void) {
    SystemInitializer initializer;
    
    // Before initialization
    TEST_ASSERT_FALSE(initializer.isSystemHealthy());
    
    // After initialization
    if (initializer.initializeSystem()) {
        TEST_ASSERT_TRUE(initializer.isSystemHealthy());
        initializer.shutdownSystem();
    }
}

void test_system_components_access(void) {
    SystemInitializer initializer;
    
    if (initializer.initializeSystem()) {
        // Test component access
        MultiCoreCar* motorCar = initializer.getMultiCoreCar();
        ROSCommTask* rosTask = initializer.getROSCommTask();
        InterCoreCommunication* interCore = initializer.getInterCoreCommunication();
        
        TEST_ASSERT_NOT_NULL(motorCar);
        TEST_ASSERT_NOT_NULL(rosTask);
        TEST_ASSERT_NOT_NULL(interCore);
        
        initializer.shutdownSystem();
    }
}

void test_global_system_functions(void) {
    // Test global system functions
    TEST_ASSERT_FALSE(isSystemHealthy()); // Should be false before initialization
    
    if (initializeSystem()) {
        TEST_ASSERT_TRUE(isSystemHealthy());
        
        // Test component access
        MultiCoreCar* motorCar = getMultiCoreCar();
        ROSCommTask* rosTask = getROSCommTask();
        InterCoreCommunication* interCore = getInterCoreCommunication();
        
        TEST_ASSERT_NOT_NULL(motorCar);
        TEST_ASSERT_NOT_NULL(rosTask);
        TEST_ASSERT_NOT_NULL(interCore);
        
        shutdownSystem();
        TEST_ASSERT_FALSE(isSystemHealthy());
    }
}

void test_system_status_printing(void) {
    SystemInitializer initializer;
    
    if (initializer.initializeSystem()) {
        // Should not crash
        initializer.printSystemStatus();
        printSystemStatus();
        
        initializer.shutdownSystem();
    }
}

void test_error_handling(void) {
    SystemInitializer initializer;
    
    // Test error reporting
    initializer.reportInitializationError("TEST", "Test error");
    TEST_ASSERT_NOT_EQUAL(0, initializer.getInitializationErrors());
    
    // Clear errors
    initializer.clearInitializationErrors();
    TEST_ASSERT_EQUAL(0, initializer.getInitializationErrors());
}

void test_multiple_initialization_attempts(void) {
    SystemInitializer initializer;
    
    // First initialization
    bool firstInit = initializer.initializeSystem();
    
    // Second initialization should return true (already initialized)
    bool secondInit = initializer.initializeSystem();
    
    if (firstInit) {
        TEST_ASSERT_TRUE(secondInit);
        initializer.shutdownSystem();
    }
}

void test_shutdown_after_initialization(void) {
    SystemInitializer initializer;
    
    if (initializer.initializeSystem()) {
        TEST_ASSERT_TRUE(initializer.isSystemInitialized());
        
        initializer.shutdownSystem();
        TEST_ASSERT_FALSE(initializer.isSystemInitialized());
    }
}

void test_system_initializer_destructor(void) {
    // Test that destructor cleans up properly
    {
        SystemInitializer initializer;
        if (initializer.initializeSystem()) {
            TEST_ASSERT_TRUE(initializer.isSystemInitialized());
        }
        // Destructor should clean up automatically
    }
    
    // Should not crash
    TEST_ASSERT_TRUE(true);
}

int runUnityTests_system_init(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_system_initializer_creation);
    RUN_TEST(test_configuration_validation);
    RUN_TEST(test_system_health_check);
    RUN_TEST(test_system_components_access);
    RUN_TEST(test_global_system_functions);
    RUN_TEST(test_system_status_printing);
    RUN_TEST(test_error_handling);
    RUN_TEST(test_multiple_initialization_attempts);
    RUN_TEST(test_shutdown_after_initialization);
    RUN_TEST(test_system_initializer_destructor);
    
    return UNITY_END();
}
