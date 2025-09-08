#ifndef ROS_COMM_TASK_H
#define ROS_COMM_TASK_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "system_config.h"
#include "inter_core_comm.h"
#include "task_performance.h"
#include "mutex_wrapper.h"
#include "time_utils.h"
#include "validation_utils.h"
#include "error_handler.h"
#include "initializable.h"

// ROS2 includes
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// ROS2 message includes
#include <geometry_msg/msg/vehicle_geometry.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <nav_msgs/msg/odometry.h>

// USB CDC includes
#include "USB.h"
#include "USBCDC.h"

// External USB Serial declaration
extern USBCDC USBSerial;

// ROS Logger includes
#include "ros_logger.h"

// =============================================================================
// ROS COMMUNICATION TASK CLASS
// =============================================================================

class ROSCommTask : public TaskInitializable {
public:
    // Constructor and destructor
    ROSCommTask();
    ~ROSCommTask();
    
    // Initialization and cleanup
    bool initialize(ThreadSafeCarState* carState, const SystemConfig_t& config);
    void cleanup();
    
    // Task management
    bool startTask();
    void stopTask();
    bool isRunning();
    
    // Performance monitoring
    ROSCommPerformanceMetrics getPerformanceMetrics();
    void resetPerformanceMetrics();
    
    // Health monitoring
    bool isHealthy();
    uint32_t getErrorCount();
    
    // Connection management
    bool isAgentConnected();
    void forceReconnection();
    
protected:
    // Virtual method implementations from TaskInitializable
    virtual bool doInitialize() override;
    virtual void doCleanup() override;
    virtual bool doHealthCheck() const override;
    virtual bool createTask() override;
    virtual void destroyTask() override;
    virtual void taskFunction() override;
    
private:
    // ROS2 objects
    rclc_support_t support;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_executor_t executor;
    
    // Publishers
    rcl_publisher_t motor_rpm_publisher;
    std_msgs__msg__Float32MultiArray motor_rpm_msg;
    
    // Subscribers
    rcl_subscription_t twist_subscriber;
    rcl_subscription_t geom_subscriber;
    rcl_subscription_t odom_subscriber;
    
    // Messages
    geometry_msg__msg__VehicleGeometry geom_msg;
    geometry_msgs__msg__Twist twist_msg;
    nav_msgs__msg__Odometry odom_msg_storage;
    
    // Timers
    rcl_timer_t motor_rpm_timer;
    
    // Inter-core communication
    ThreadSafeCarState* carState;
    
    // Task management
    TaskHandle_t taskHandle;
    
    // Timing control
    TickType_t lastWakeTime;
    const TickType_t taskPeriodTicks;
    
    // Performance monitoring
    ROSCommPerformanceMetrics performance;
    
    // System configuration
    SystemConfig_t config;
    
    // Agent connection management
    typedef enum {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    } agent_state_t;
    
    volatile agent_state_t agentState;
    MutexWrapper stateMutex;
    uint32_t lastAgentPingTime;
    uint32_t reconnectionAttempts;
    
    // USB CDC communication
    USBCDC* usbSerial;
    
    // Private methods
    void executeTaskLoop();
    bool createROS2Entities();
    void destroyROS2Entities();
    void handleAgentConnection();
    void processROS2Messages();
    void publishMotorStatus();
    void handleTwistMessage(const geometry_msgs__msg__Twist* twist);
    void handleGeometryMessage(const geometry_msg__msg__VehicleGeometry* geom);
    void handleOdometryMessage(const nav_msgs__msg__Odometry* odom);
    
    // Agent connection management
    bool pingAgent();
    void updateAgentState();
    void attemptReconnection();
    
    bool isDeadlineMissed();
    
    // Static callback functions for ROS2
    static void motor_rpm_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
    static void twist_callback(const void* msgin);
    static void vehicle_geometry_callback(const void* msgin);
    static void odom_callback(const void* msgin);
    
    // Static instance pointer for callbacks
    static ROSCommTask* instance;
};

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Calculate steering angle from twist message
float calculateSteeringAngle(float omega, float vel, float wheelbase);

#endif // ROS_COMM_TASK_H
