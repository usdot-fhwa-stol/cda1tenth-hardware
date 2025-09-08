#include <unity.h>
#include <Arduino.h>

// Test function declarations
extern int runUnityTests_inter_core_comm(void);
extern int runUnityTests_ros_logger(void);
extern int runUnityTests_system_config(void);
extern int runUnityTests_system_init(void);
extern int runUnityTests_ros_comm_task(void);

void setUp(void) {
    // Set up test environment
}

void tearDown(void) {
    // Clean up after test
}

void test_all_modules(void) {
    // Run all module tests
    int result = 0;
    
    result += runUnityTests_inter_core_comm();
    result += runUnityTests_ros_logger();
    result += runUnityTests_system_config();
    result += runUnityTests_system_init();
    result += runUnityTests_ros_comm_task();
    
    TEST_ASSERT_EQUAL(0, result);
}

int runUnityTests(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_all_modules);
    
    return UNITY_END();
}

void setup() {
    delay(2000);
    runUnityTests();
}

void loop() {
    // Empty loop
}
