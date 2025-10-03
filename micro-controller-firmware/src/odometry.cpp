#include "odometry.h"
#include <math.h>
#include <Arduino.h>
#include <rclc/executor.h>

// External sensor data (from main.cpp)
extern struct SensorData {
    float gyro_z;
    float right_rpm;
    float left_rpm;
    uint32_t last_update_ms;
    SemaphoreHandle_t mutex;
} sensorData;

// Global odometry state
OdometryState odom_state = {0.0f, 0.0f, 0.0f, 0.03f, 0, 0};
OdometryConfig odom_config = {100, 0.03f}; // 10Hz, 3cm radius

// Debug counters for publish status
uint32_t odom_publish_success_count = 0;
uint32_t odom_publish_failure_count = 0;
rcl_ret_t publisher_init_result = RCL_RET_OK;
uint32_t odometry_callback_count = 0;
rcl_ret_t last_publish_error_code = RCL_RET_OK;

// ROS2 objects
rcl_publisher_t odom_publisher;
rcl_timer_t odom_timer;

// Helper function to convert yaw to quaternion
geometry_msgs__msg__Quaternion yaw_to_quaternion(float yaw_rad) {
    geometry_msgs__msg__Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = sinf(yaw_rad / 2.0f);
    q.w = cosf(yaw_rad / 2.0f);
    return q;
}

// External car initialization flag
extern bool car_initialized;

// Odometry timer callback
void odometry_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (!timer || !car_initialized) return;
    
    // Increment publish counter
    odom_state.publish_count++;
    
    // Debug: Simple counter to verify callback is being called
    odometry_callback_count++;
    
    uint32_t now_ms = millis();
    float dt = (odom_state.last_update_ms == 0) ? 0.0f : (now_ms - odom_state.last_update_ms) / 1000.0f;
    odom_state.last_update_ms = now_ms;

    // Get cached sensor data (non-blocking)
    float cached_gyro_z = 0.0f;
    float cached_right_rpm = 0.0f;
    float cached_left_rpm = 0.0f;
    
    if (xSemaphoreTake(sensorData.mutex, 0) == pdTRUE) {
        cached_gyro_z = sensorData.gyro_z;
        cached_right_rpm = sensorData.right_rpm;
        cached_left_rpm = sensorData.left_rpm;
        xSemaphoreGive(sensorData.mutex);
    }
    
    // Update odometry state
    odom_state.yaw += cached_gyro_z * dt;
    
    float avg_rpm = 0.5f * (cached_right_rpm + cached_left_rpm);
    float v = (avg_rpm / 60.0f) * 2.0f * M_PI * odom_state.wheel_radius;

    odom_state.x += v * cosf(odom_state.yaw) * dt;
    odom_state.y += v * sinf(odom_state.yaw) * dt;

    // Use static message to avoid repeated allocation/deallocation
    static nav_msgs__msg__Odometry odom;
    static bool odom_initialized = false;
    static char frame_id_buffer[16] = "odom";
    static char child_frame_id_buffer[16] = "base_link";
    
    if (!odom_initialized) {
        nav_msgs__msg__Odometry__init(&odom);
        // Use static buffers instead of dynamic allocation
        odom.header.frame_id.data = frame_id_buffer;
        odom.header.frame_id.size = 4;
        odom.header.frame_id.capacity = 16;
        odom.child_frame_id.data = child_frame_id_buffer;
        odom.child_frame_id.size = 9;
        odom.child_frame_id.capacity = 16;
        odom_initialized = true;
    }
    
    // Set frame IDs for each message (as shown in GitHub issue)
    odom.header.frame_id.data = frame_id_buffer;
    odom.header.frame_id.size = 4;
    odom.child_frame_id.data = child_frame_id_buffer;
    odom.child_frame_id.size = 9;

    // Update odometry message with simplified values
    odom.pose.pose.position.x = odom_state.x;
    odom.pose.pose.position.y = odom_state.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = yaw_to_quaternion(odom_state.yaw);
    
    // Test with minimal values to see if the issue is with the data
    static uint32_t test_counter = 0;
    test_counter++;
    
    // Use simple test values every 10th message
    if (test_counter % 10 == 0) {
        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.w = 1.0;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = 0.0;
    }

    // Set covariance to zero for simplicity
    for (int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = 0.1;  // x variance
    odom.pose.covariance[7] = 0.1;  // y variance
    odom.pose.covariance[35] = 0.1; // yaw variance

    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = cached_gyro_z;
    
    // Set timestamp for odometry message
    // Use millis() for timestamp
    uint32_t time_ms = millis();
    odom.header.stamp.sec = time_ms / 1000;
    odom.header.stamp.nanosec = (time_ms % 1000) * 1000000;
    
    // Check if publisher is valid before publishing
    if (odom_publisher.impl == NULL) {
        // Publisher not initialized, skip publishing
        odom_publish_failure_count++;
        static uint32_t publisher_null_count = 0;
        publisher_null_count++;
        return;
    }
    
    // Validate message content before publishing
    if (odom.header.frame_id.data == NULL || odom.child_frame_id.data == NULL) {
        // Message not properly initialized
        odom_publish_failure_count++;
        static uint32_t frame_id_null_count = 0;
        frame_id_null_count++;
        return;
    }
    
    // Publish odometry message
    rcl_ret_t ret = rcl_publish(&odom_publisher, &odom, nullptr);
    if (ret != RCL_RET_OK) {
        // Store error for later handling (avoid blocking in callback)
        static rcl_ret_t last_publish_error = RCL_RET_OK;
        last_publish_error = ret;
        
        // Debug: Store specific error details
        static uint32_t error_count = 0;
        error_count++;
        
        // Store the specific error code for debugging
        last_publish_error_code = ret;
        
        // Debug: Try a simple test message every 100th attempt
        static uint32_t test_attempts = 0;
        test_attempts++;
        if (test_attempts % 100 == 0) {
            // Create a minimal test message
            static nav_msgs__msg__Odometry test_odom;
            static bool test_odom_init = false;
            if (!test_odom_init) {
                nav_msgs__msg__Odometry__init(&test_odom);
                test_odom_init = true;
            }
            
            // Set minimal test values
            test_odom.pose.pose.position.x = 0.0;
            test_odom.pose.pose.position.y = 0.0;
            test_odom.pose.pose.position.z = 0.0;
            test_odom.pose.pose.orientation.w = 1.0;
            test_odom.pose.pose.orientation.x = 0.0;
            test_odom.pose.pose.orientation.y = 0.0;
            test_odom.pose.pose.orientation.z = 0.0;
            
            // Set frame IDs
            static char test_frame[16] = "odom";
            static char test_child[16] = "base_link";
            test_odom.header.frame_id.data = test_frame;
            test_odom.header.frame_id.size = 4;
            test_odom.child_frame_id.data = test_child;
            test_odom.child_frame_id.size = 9;
            
            // Set timestamp
            test_odom.header.stamp.sec = time_ms / 1000;
            test_odom.header.stamp.nanosec = (time_ms % 1000) * 1000000;
            
            // Try test publish
            rcl_ret_t test_ret = rcl_publish(&odom_publisher, &test_odom, nullptr);
            if (test_ret == RCL_RET_OK) {
                odom_publish_success_count++;
            }
        }
    } else {
        // Message was successfully queued for transmission
        // This doesn't guarantee it reached the network, but it's a good sign
        odom_publish_success_count++;
    }
    
    // Debug: Store publish status for monitoring
    static rcl_ret_t last_publish_status = RCL_RET_OK;
    last_publish_status = ret;
    
    // Debug: Track publish success/failure
    if (ret == RCL_RET_OK) {
        odom_publish_success_count++;
        // Simple debug: increment a counter every time we successfully publish
        static uint32_t total_published = 0;
        total_published++;
    } else {
        odom_publish_failure_count++;
        // Store the specific error code for debugging
        last_publish_error_code = ret;
    }
}

// Initialize odometry
bool odometry_init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    // Try to initialize the odometry publisher with best_effort for performance
    // Based on GitHub issue #148, we need to use best_effort with proper configuration
    rcl_ret_t ret = rclc_publisher_init_best_effort(
          &odom_publisher, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
          "odom");
    
    // Debug: Store publisher init result
    publisher_init_result = ret;
    
    if (ret != RCL_RET_OK) {
        // Publisher initialization failed
        static rcl_ret_t last_init_error = RCL_RET_OK;
        last_init_error = ret;
        return false;
    }

    // Ensure period is set before timer initialization
    if (odom_config.period_ms == 0) {
        odom_config.period_ms = 100; // Default to 10Hz (slower for stability)
    }

    rcl_ret_t timer_ret = rclc_timer_init_default(
          &odom_timer, support, RCL_MS_TO_NS(odom_config.period_ms), odometry_timer_callback);
    if (timer_ret != RCL_RET_OK) {
        // Timer initialization failed
        static rcl_ret_t last_timer_error = RCL_RET_OK;
        last_timer_error = timer_ret;
        return false;
    }

    rcl_ret_t executor_ret = rclc_executor_add_timer(executor, &odom_timer);
    if (executor_ret != RCL_RET_OK) {
        // Executor add timer failed
        static rcl_ret_t last_executor_error = RCL_RET_OK;
        last_executor_error = executor_ret;
        return false;
    }

    odom_state.last_update_ms = 0;
    
    // Send a test odometry message on boot to verify publisher works
    send_test_odometry_message();
    
    // Also try a simple immediate publish test
    static nav_msgs__msg__Odometry test_msg;
    static bool test_msg_initialized = false;
    if (!test_msg_initialized) {
        nav_msgs__msg__Odometry__init(&test_msg);
        test_msg_initialized = true;
    }
    
    // Set test values
    test_msg.pose.pose.position.x = 0.0;
    test_msg.pose.pose.position.y = 0.0;
    test_msg.pose.pose.position.z = 0.0;
    test_msg.pose.pose.orientation.w = 1.0;
    test_msg.pose.pose.orientation.x = 0.0;
    test_msg.pose.pose.orientation.y = 0.0;
    test_msg.pose.pose.orientation.z = 0.0;
    
    // Set frame IDs for test message
    static char test_frame_id[16] = "odom";
    static char test_child_frame_id[16] = "base_link";
    test_msg.header.frame_id.data = test_frame_id;
    test_msg.header.frame_id.size = 4;
    test_msg.header.frame_id.capacity = 16;
    test_msg.child_frame_id.data = test_child_frame_id;
    test_msg.child_frame_id.size = 9;
    test_msg.child_frame_id.capacity = 16;
    
    // Set timestamp
    uint32_t test_time_ms = millis();
    test_msg.header.stamp.sec = test_time_ms / 1000;
    test_msg.header.stamp.nanosec = (test_time_ms % 1000) * 1000000;
    
    // Try immediate publish test
    rcl_ret_t test_ret = rcl_publish(&odom_publisher, &test_msg, nullptr);
    if (test_ret != RCL_RET_OK) {
        // Test publish failed - store error
        last_publish_error_code = test_ret;
    } else {
        // Test publish succeeded
        odom_publish_success_count++;
    }
    
    return true;
}

// Finalize odometry
void odometry_fini(rcl_node_t* node) {
    rcl_ret_t ret1 = rcl_timer_fini(&odom_timer);
    rcl_ret_t ret2 = rcl_publisher_fini(&odom_publisher, node);
    if (ret1 != RCL_RET_OK || ret2 != RCL_RET_OK) {
        // Handle cleanup errors if needed
    }
    
    // Clean up odometry message to prevent memory leaks
    static nav_msgs__msg__Odometry odom;
    nav_msgs__msg__Odometry__fini(&odom);
}

// Reset odometry state
void odometry_reset(float x, float y, float yaw_rad) {
    odom_state.x = x;
    odom_state.y = y;
    odom_state.yaw = yaw_rad;
    odom_state.last_update_ms = 0;
}

// Set odometry period
void odometry_set_period_ms(unsigned int period_ms) {
    odom_config.period_ms = period_ms;
    // Note: Timer recreation would need to be handled in the main loop
}

// Set wheel radius
void odometry_set_wheel_radius(float radius) {
    odom_state.wheel_radius = radius;
    odom_config.wheel_radius = radius;
}

// Send a test odometry message to verify publisher works
void send_test_odometry_message() {
    // Create a test odometry message
    static nav_msgs__msg__Odometry test_odom;
    static bool test_odom_initialized = false;
    static char frame_id_buffer[16] = "odom";
    static char child_frame_id_buffer[16] = "base_link";
    
    if (!test_odom_initialized) {
        nav_msgs__msg__Odometry__init(&test_odom);
        test_odom.header.frame_id.data = frame_id_buffer;
        test_odom.header.frame_id.size = 4;
        test_odom.header.frame_id.capacity = 16;
        test_odom.child_frame_id.data = child_frame_id_buffer;
        test_odom.child_frame_id.size = 9;
        test_odom.child_frame_id.capacity = 16;
        test_odom_initialized = true;
    }
    
    // Set test values
    test_odom.pose.pose.position.x = 0.0;
    test_odom.pose.pose.position.y = 0.0;
    test_odom.pose.pose.position.z = 0.0;
    test_odom.pose.pose.orientation = yaw_to_quaternion(0.0);
    
    test_odom.pose.covariance[0]  = 0.2;
    test_odom.pose.covariance[7]  = 0.2;
    test_odom.pose.covariance[35] = 0.4;
    
    test_odom.twist.twist.linear.x = 0.0;
    test_odom.twist.twist.linear.y = 0.0;
    test_odom.twist.twist.linear.z = 0.0;
    test_odom.twist.twist.angular.x = 0.0;
    test_odom.twist.twist.angular.y = 0.0;
    test_odom.twist.twist.angular.z = 0.0;
    
    // Publish test message
    rcl_ret_t ret = rcl_publish(&odom_publisher, &test_odom, nullptr);
    
    // Store result for debugging
    static rcl_ret_t test_publish_result = RCL_RET_OK;
    test_publish_result = ret;
}
