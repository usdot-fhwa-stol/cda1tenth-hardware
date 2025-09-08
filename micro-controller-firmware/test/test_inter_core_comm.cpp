#include <unity.h>
#include <Arduino.h>
#include "inter_core_comm.h"

void test_motor_command_validation(void) {
    MotorCommand_t cmd;
    motor_command_init(&cmd);
    
    // Valid command
    cmd.steering_angle = 10.0f;
    cmd.drive_speed = 1.0f;
    cmd.wheelbase = 0.185f;
    cmd.track_width = 0.15f;
    TEST_ASSERT_TRUE(validateMotorCommand(cmd));
    
    // Invalid steering angle
    cmd.steering_angle = 50.0f; // > 45 degrees
    TEST_ASSERT_FALSE(validateMotorCommand(cmd));
    
    // Invalid speed
    cmd.steering_angle = 10.0f;
    cmd.drive_speed = 10.0f; // > 5 m/s
    TEST_ASSERT_FALSE(validateMotorCommand(cmd));
    
    // Invalid wheelbase
    cmd.drive_speed = 1.0f;
    cmd.wheelbase = 0.0f; // <= 0
    TEST_ASSERT_FALSE(validateMotorCommand(cmd));
}

void test_motor_status_validation(void) {
    MotorStatus_t status;
    motor_status_init(&status);
    
    // Valid status
    status.right_motor_rpm = 100.0f;
    status.left_motor_rpm = 100.0f;
    status.steering_angle = 10.0f;
    TEST_ASSERT_TRUE(validateMotorStatus(status));
    
    // Invalid RPM
    status.right_motor_rpm = 3000.0f; // > 2000
    TEST_ASSERT_FALSE(validateMotorStatus(status));
    
    // Invalid steering angle
    status.right_motor_rpm = 100.0f;
    status.steering_angle = 50.0f; // > 45 degrees
    TEST_ASSERT_FALSE(validateMotorStatus(status));
}

void test_thread_safe_car_state_initialization(void) {
    ThreadSafeCarState carState;
    TEST_ASSERT_TRUE(carState.initialize());
    TEST_ASSERT_TRUE(carState.isHealthy());
}

void test_command_queue_operations(void) {
    ThreadSafeCarState carState;
    carState.initialize();
    
    MotorCommand_t cmd;
    motor_command_init(&cmd);
    cmd.steering_angle = 10.0f;
    cmd.drive_speed = 1.0f;
    
    // Send command
    TEST_ASSERT_TRUE(carState.sendCommandToQueue(cmd));
    
    // Receive command
    MotorCommand_t receivedCmd;
    TEST_ASSERT_TRUE(carState.receiveCommandFromQueue(receivedCmd, pdMS_TO_TICKS(100)));
    TEST_ASSERT_EQUAL_FLOAT(cmd.steering_angle, receivedCmd.steering_angle);
    TEST_ASSERT_EQUAL_FLOAT(cmd.drive_speed, receivedCmd.drive_speed);
}

void test_status_queue_operations(void) {
    ThreadSafeCarState carState;
    carState.initialize();
    
    MotorStatus_t status;
    motor_status_init(&status);
    status.right_motor_rpm = 100.0f;
    status.left_motor_rpm = 100.0f;
    
    // Send status
    TEST_ASSERT_TRUE(carState.sendStatusToQueue(status));
    
    // Receive status
    MotorStatus_t receivedStatus;
    TEST_ASSERT_TRUE(carState.receiveStatusFromQueue(receivedStatus, pdMS_TO_TICKS(100)));
    TEST_ASSERT_EQUAL_FLOAT(status.right_motor_rpm, receivedStatus.right_motor_rpm);
    TEST_ASSERT_EQUAL_FLOAT(status.left_motor_rpm, receivedStatus.left_motor_rpm);
}

void test_inter_core_communication_singleton(void) {
    InterCoreCommunication& comm1 = InterCoreCommunication::getInstance();
    InterCoreCommunication& comm2 = InterCoreCommunication::getInstance();
    
    // Should return the same instance
    TEST_ASSERT_EQUAL_PTR(&comm1, &comm2);
}

void test_inter_core_communication_initialization(void) {
    InterCoreCommunication& comm = InterCoreCommunication::getInstance();
    TEST_ASSERT_TRUE(comm.initialize());
    TEST_ASSERT_TRUE(comm.isHealthy());
}

void test_queue_overflow_handling(void) {
    ThreadSafeCarState carState;
    carState.initialize();
    
    MotorCommand_t cmd;
    motor_command_init(&cmd);
    cmd.steering_angle = 10.0f;
    cmd.drive_speed = 1.0f;
    
    // Fill queue to capacity
    for (int i = 0; i < 10; i++) {
        carState.sendCommandToQueue(cmd);
    }
    
    // Queue should be full, but system should handle gracefully
    TEST_ASSERT_TRUE(carState.isCommandQueueFull());
}

void test_timestamp_generation(void) {
    uint32_t timestamp1 = getCurrentTimestampUs();
    delay(1); // Small delay
    uint32_t timestamp2 = getCurrentTimestampUs();
    
    TEST_ASSERT_TRUE(timestamp2 > timestamp1);
}

int runUnityTests_inter_core_comm(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_motor_command_validation);
    RUN_TEST(test_motor_status_validation);
    RUN_TEST(test_thread_safe_car_state_initialization);
    RUN_TEST(test_command_queue_operations);
    RUN_TEST(test_status_queue_operations);
    RUN_TEST(test_inter_core_communication_singleton);
    RUN_TEST(test_inter_core_communication_initialization);
    RUN_TEST(test_queue_overflow_handling);
    RUN_TEST(test_timestamp_generation);
    
    return UNITY_END();
}
