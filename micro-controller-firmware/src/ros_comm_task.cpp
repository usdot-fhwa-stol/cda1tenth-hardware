#include "ros_comm_task.h"
#include <math.h>
#include <stdio.h>

// Static instance pointer for callbacks
ROSCommTask* ROSCommTask::instance = nullptr;

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

float calculateSteeringAngle(float omega, float vel, float wheelbase) {
    if (omega == 0.0f || vel == 0.0f) {
        return 0.0f;
    }
    
    // Use bicycle model: tan(steering_angle) = wheelbase * omega / vel
    float steering_angle_rad = atanf(wheelbase * omega / vel);
    return steering_angle_rad * (180.0f / M_PI); // Convert to degrees
}

// =============================================================================
// ROS COMMUNICATION TASK IMPLEMENTATION
// =============================================================================

ROSCommTask::ROSCommTask() 
    : TaskInitializable("ROSCommTask"),
      carState(nullptr), taskHandle(nullptr),
      lastWakeTime(0), taskPeriodTicks(pdMS_TO_TICKS(ROS_TASK_PERIOD_MS)), // Use config constant
      agentState(WAITING_AGENT), stateMutex("ROSState"), lastAgentPingTime(0), reconnectionAttempts(0),
      usbSerial(nullptr) {
    
    // Initialize performance metrics
    performance.init();
    
    // Initialize ROS2 objects
    // Note: support will be initialized in createROS2Entities()
    node = rcl_get_zero_initialized_node();
    allocator = rcl_get_default_allocator();
    executor = rclc_executor_get_zero_initialized_executor();
    
    // Initialize publishers and subscribers
    motor_rpm_publisher = rcl_get_zero_initialized_publisher();
    twist_subscriber = rcl_get_zero_initialized_subscription();
    geom_subscriber = rcl_get_zero_initialized_subscription();
    odom_subscriber = rcl_get_zero_initialized_subscription();
    motor_rpm_timer = rcl_get_zero_initialized_timer();
    
    // Initialize messages
    std_msgs__msg__Float32MultiArray__init(&motor_rpm_msg);
    geometry_msg__msg__VehicleGeometry__init(&geom_msg);
    geometry_msgs__msg__Twist__init(&twist_msg);
    nav_msgs__msg__Odometry__init(&odom_msg_storage);
}

ROSCommTask::~ROSCommTask() {
    cleanup();
}

bool ROSCommTask::initialize(ThreadSafeCarState* carState, const SystemConfig_t& config) {
    // Store references
    this->carState = carState;
    this->config = config;
    
    // Initialize USB CDC
    usbSerial = &USBSerial;
    
    // Initialize motor RPM message
    motor_rpm_msg.data.size = 3;
    motor_rpm_msg.data.capacity = 3;
    motor_rpm_msg.data.data = (float*)malloc(3 * sizeof(float));
    if (motor_rpm_msg.data.data == nullptr) {
        return false;
    }
    
    // Set static instance for callbacks
    instance = this;
    
    // Use base class initialization
    return Initializable::initialize();
}

void ROSCommTask::cleanup() {
    Initializable::cleanup();
}

// Virtual method implementations from TaskInitializable
bool ROSCommTask::doInitialize() {
    return true; // State mutex is already created in constructor
}

void ROSCommTask::doCleanup() {
    // Clean up ROS2 entities
    destroyROS2Entities();
    
    // Free motor RPM message data
    if (motor_rpm_msg.data.data != nullptr) {
        free(motor_rpm_msg.data.data);
        motor_rpm_msg.data.data = nullptr;
    }
    
    // Finalize ROS2 messages
    std_msgs__msg__Float32MultiArray__fini(&motor_rpm_msg);
    geometry_msg__msg__VehicleGeometry__fini(&geom_msg);
    geometry_msgs__msg__Twist__fini(&twist_msg);
    nav_msgs__msg__Odometry__fini(&odom_msg_storage);
    
    instance = nullptr;
}

bool ROSCommTask::doHealthCheck() const {
    return isTaskHealthy();
}

bool ROSCommTask::createTask() {
    // Create the task
    BaseType_t result = xTaskCreatePinnedToCore(
        staticTaskFunction,
        "ROSCommTask",
        ROS_COMM_TASK_STACK_SIZE,
        this,
        ROS_COMM_TASK_PRIORITY,
        &taskHandle,
        ROS_TASK_CORE_ID
    );
    
    if (result != pdPASS) {
        return false;
    }
    
    lastWakeTime = xTaskGetTickCount();
    return true;
}

void ROSCommTask::destroyTask() {
    if (taskHandle != nullptr) {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }
}

void ROSCommTask::taskFunction() {
    // Initialize timing
    lastWakeTime = xTaskGetTickCount();
    
    while (isTaskRunning()) {
        uint32_t startTime = TimeUtils::getCurrentTimestampUs();
        
        // Execute task loop
        executeTaskLoop();
        
        uint32_t executionTime = TimeUtils::getCurrentTimestampUs() - startTime;
        performance.update(executionTime);
        
        // Check for deadline miss
        if (isDeadlineMissed()) {
            performance.missed_deadlines++;
            taskErrorHandler.handleError("Task deadline missed");
        }
        
        // Wait for next period
        vTaskDelayUntil(&lastWakeTime, taskPeriodTicks);
    }
}

bool ROSCommTask::startTask() {
    return TaskInitializable::startTask();
}

void ROSCommTask::stopTask() {
    TaskInitializable::stopTask();
}

bool ROSCommTask::isRunning() {
    return isTaskRunning();
}


void ROSCommTask::executeTaskLoop() {
    // Handle agent connection management
    handleAgentConnection();
    
    // Process ROS2 messages if connected
    if (agentState == AGENT_CONNECTED) {
        processROS2Messages();
    }
}

bool ROSCommTask::createROS2Entities() {
    // Create init_options
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Create node
    ret = rclc_node_init_default(&node, "car_controller", "", &support);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Create motor RPM publisher
    ret = rclc_publisher_init_best_effort(
        &motor_rpm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_rpms");
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Create subscriber for vehicle geometry
    ret = rclc_subscription_init_best_effort(
        &geom_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msg, msg, VehicleGeometry),
        "vehicle_geometry");
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Create subscriber for twist messages (cmd_vel)
    ret = rclc_subscription_init_best_effort(
        &twist_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Create subscriber for odometry
    ret = rclc_subscription_init_best_effort(
        &odom_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom");
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Create motor RPM timer using configuration constant
    const unsigned int motor_rpm_timer_timeout = ROS_MOTOR_RPM_PUBLISH_PERIOD_MS;
    ret = rclc_timer_init_default(
        &motor_rpm_timer,
        &support,
        RCL_MS_TO_NS(motor_rpm_timer_timeout),
        motor_rpm_timer_callback);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Initialize executor
    unsigned int number_of_handles = 4; // 3 subscribers + 1 timer
    executor = rclc_executor_get_zero_initialized_executor();
    ret = rclc_executor_init(&executor, &support.context, number_of_handles, &allocator);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Add timer and subscriptions to executor
    ret = rclc_executor_add_timer(&executor, &motor_rpm_timer);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    ret = rclc_executor_add_subscription(&executor, &geom_subscriber, &geom_msg,
        &vehicle_geometry_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    ret = rclc_executor_add_subscription(
        &executor, &twist_subscriber, &twist_msg,
        &twist_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    ret = rclc_executor_add_subscription(
        &executor, &odom_subscriber, &odom_msg_storage,
        &odom_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Set executor timeout
    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(config.ros_spin_timeout_ms));
    
    // Initialize ROS logger
    if (!initializeROSLogger(&node, config)) {
        // Logger initialization failed, but we can continue without it
    }
    
    return true;
}

void ROSCommTask::destroyROS2Entities() {
    // Clean up ROS logger
    cleanupROSLogger();
    
    // Clean up ROS2 entities
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    if (rmw_context != nullptr) {
        (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    }
    
    (void)rcl_publisher_fini(&motor_rpm_publisher, &node);
    (void)rcl_subscription_fini(&odom_subscriber, &node);
    (void)rcl_subscription_fini(&geom_subscriber, &node);
    (void)rcl_subscription_fini(&twist_subscriber, &node);
    (void)rcl_timer_fini(&motor_rpm_timer);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

void ROSCommTask::handleAgentConnection() {
    uint32_t currentTime = TimeUtils::getCurrentTimestampMs();
    
    switch (agentState) {
        case WAITING_AGENT:
            if (currentTime - lastAgentPingTime >= config.agent_ping_interval_ms) {
                if (pingAgent()) {
                    agentState = AGENT_AVAILABLE;
                }
                lastAgentPingTime = currentTime;
            }
            break;
            
        case AGENT_AVAILABLE:
            if (createROS2Entities()) {
                agentState = AGENT_CONNECTED;
                reconnectionAttempts = 0;
            } else {
                agentState = WAITING_AGENT;
                performance.agent_reconnection_attempts++;
            }
            break;
            
        case AGENT_CONNECTED:
            if (currentTime - lastAgentPingTime >= config.agent_ping_interval_ms) {
                if (!pingAgent()) {
                    agentState = AGENT_DISCONNECTED;
                    performance.ros_connection_drops++;
                }
                lastAgentPingTime = currentTime;
            }
            break;
            
        case AGENT_DISCONNECTED:
            destroyROS2Entities();
            agentState = WAITING_AGENT;
            break;
    }
}

void ROSCommTask::processROS2Messages() {
    if (agentState != AGENT_CONNECTED) {
        return;
    }
    
    // Spin the executor to process incoming messages
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(config.ros_spin_timeout_ms));
    if (ret != RCL_RET_OK) {
        taskErrorHandler.handleError("ROS executor spin failed");
        agentState = AGENT_DISCONNECTED;
    }
}

void ROSCommTask::publishMotorStatus() {
    if (agentState != AGENT_CONNECTED || carState == nullptr) {
        return;
    }
    
    // Get motor status from multi-core system
    MotorStatus_t status;
    if (carState->getStatus(status)) {
        // Set up the message
        motor_rpm_msg.data.data[0] = status.right_motor_rpm;
        motor_rpm_msg.data.data[1] = status.left_motor_rpm;
        motor_rpm_msg.data.data[2] = status.steering_angle;
        
        // Publish motor RPM data
        rcl_ret_t ret = rcl_publish(&motor_rpm_publisher, &motor_rpm_msg, NULL);
        if (ret != RCL_RET_OK) {
            performance.ros_message_send_failures++;
            taskErrorHandler.handleError("Failed to publish motor status");
        }
    }
}

void ROSCommTask::handleTwistMessage(const geometry_msgs__msg__Twist* twist) {
    if (!ROSValidation::validateTwistMessage(twist) || carState == nullptr) {
        return;
    }
    
    performance.ros_message_receive_count++;
    
    // Calculate steering angle using bicycle model
    const float wheelbase = DEFAULT_WHEELBASE; // Use configuration constant
    float steering_angle_deg = calculateSteeringAngle(twist->angular.z, twist->linear.x, wheelbase);
    
    // Clamp steering angle to maximum using configuration constant
    if (steering_angle_deg > MAX_STEERING_ANGLE_DEG) {
        steering_angle_deg = MAX_STEERING_ANGLE_DEG;
    } else if (steering_angle_deg < -MAX_STEERING_ANGLE_DEG) {
        steering_angle_deg = -MAX_STEERING_ANGLE_DEG;
    }
    
    // Create motor command for multi-core system
    MotorCommand_t cmd;
    motor_command_init(&cmd);
    cmd.steering_angle = steering_angle_deg;
    cmd.drive_speed = twist->linear.x; // Keep as m/s, conversion happens in motor task
    cmd.wheelbase = wheelbase;
    cmd.track_width = DEFAULT_TRACK_WIDTH; // Use configuration constant
    cmd.timestamp = TimeUtils::getCurrentTimestampUs();
    
    // Send command to multi-core system
    if (!carState->sendCommandToQueue(cmd)) {
        taskErrorHandler.handleError("Failed to send command to motor control queue");
    }
}

void ROSCommTask::handleGeometryMessage(const geometry_msg__msg__VehicleGeometry* geom) {
    if (!ROSValidation::validateGeometryMessage(geom)) {
        return;
    }
    
    performance.ros_message_receive_count++;
    
    // Extract wheelbase and track width from geometry message
    float x_front_axle = 0.5f * (geom->front_left_wheel_pose.position.x + geom->front_right_wheel_pose.position.x);
    float x_back_axle = 0.5f * (geom->back_left_wheel_pose.position.x + geom->back_right_wheel_pose.position.x);
    float wheelbase = fabsf(x_front_axle - x_back_axle);
    float track_width = fabsf(geom->front_left_wheel_pose.position.y - geom->front_right_wheel_pose.position.y);
    
    // Log geometry information (throttled)
    static uint32_t last_log_ms = 0;
    const uint32_t now_ms = TimeUtils::getCurrentTimestampMs();
    if (now_ms - last_log_ms >= ROS_GEOMETRY_LOG_PERIOD_MS) { // Use configuration constant
        ROS_LOG_INFOF("GEOMETRY", "Vehicle geometry - wheelbase: %.3f, track_width: %.3f", wheelbase, track_width);
        last_log_ms = now_ms;
    }
}

void ROSCommTask::handleOdometryMessage(const nav_msgs__msg__Odometry* odom) {
    if (!ROSValidation::validateOdometryMessage(odom)) {
        return;
    }
    
    performance.ros_message_receive_count++;
    
    // Extract position and orientation
    float odom_x = (float)odom->pose.pose.position.x;
    float odom_y = (float)odom->pose.pose.position.y;
    
    // Extract yaw from quaternion
    const float qx = odom->pose.pose.orientation.x;
    const float qy = odom->pose.pose.orientation.y;
    const float qz = odom->pose.pose.orientation.z;
    const float qw = odom->pose.pose.orientation.w;
    const float siny_cosp = 2.0f * (qw * qz + qx * qy);
    const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    float odom_yaw = atan2f(siny_cosp, cosy_cosp);
    
    // Throttled logging
    static uint32_t last_log_ms = 0;
    const uint32_t now_ms = TimeUtils::getCurrentTimestampMs();
    if (now_ms - last_log_ms >= ROS_ODOMETRY_LOG_PERIOD_MS) { // Use configuration constant
        const float v = (float)odom->twist.twist.linear.x;
        const float wz = (float)odom->twist.twist.angular.z;
        
        const char *frame_id = odom->header.frame_id.data ? odom->header.frame_id.data : "";
        const char *child_id = odom->child_frame_id.data ? odom->child_frame_id.data : "";
        
        ROS_LOG_INFOF("ODOMETRY", "ODOM | frame:%s child:%s | pos:(%.2f, %.2f) yaw:%.2f | v:%.2f ang:%.2f",
            frame_id, child_id, odom_x, odom_y, odom_yaw, v, wz);
        last_log_ms = now_ms;
    }
}

bool ROSCommTask::pingAgent() {
    return (RMW_RET_OK == rmw_uros_ping_agent(config.agent_ping_timeout_ms, 1));
}

void ROSCommTask::updateAgentState() {
    // This method can be used for more complex agent state management
    // For now, the state is managed in handleAgentConnection()
}

void ROSCommTask::attemptReconnection() {
    if (agentState == AGENT_DISCONNECTED) {
        agentState = WAITING_AGENT;
        reconnectionAttempts++;
        performance.agent_reconnection_attempts++;
    }
}

bool ROSCommTask::isDeadlineMissed() {
    TickType_t currentTime = xTaskGetTickCount();
    return (currentTime - lastWakeTime) > taskPeriodTicks;
}

ROSCommPerformanceMetrics ROSCommTask::getPerformanceMetrics() {
    return performance;
}

void ROSCommTask::resetPerformanceMetrics() {
    performance.reset();
}

bool ROSCommTask::isHealthy() {
    return TaskInitializable::isHealthy();
}

uint32_t ROSCommTask::getErrorCount() {
    return taskErrorHandler.getErrorCount();
}

bool ROSCommTask::isAgentConnected() {
    return agentState == AGENT_CONNECTED;
}

void ROSCommTask::forceReconnection() {
    if (agentState == AGENT_CONNECTED) {
        agentState = AGENT_DISCONNECTED;
    }
}

// =============================================================================
// STATIC CALLBACK FUNCTIONS
// =============================================================================

void ROSCommTask::motor_rpm_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void) last_call_time;
    if (instance != nullptr && timer != nullptr) {
        instance->publishMotorStatus();
    }
}

void ROSCommTask::twist_callback(const void* msgin) {
    if (instance != nullptr && msgin != nullptr) {
        const auto* twist = static_cast<const geometry_msgs__msg__Twist*>(msgin);
        instance->handleTwistMessage(twist);
    }
}

void ROSCommTask::vehicle_geometry_callback(const void* msgin) {
    if (instance != nullptr && msgin != nullptr) {
        const auto* geom = static_cast<const geometry_msg__msg__VehicleGeometry*>(msgin);
        instance->handleGeometryMessage(geom);
    }
}

void ROSCommTask::odom_callback(const void* msgin) {
    if (instance != nullptr && msgin != nullptr) {
        const auto* odom = static_cast<const nav_msgs__msg__Odometry*>(msgin);
        instance->handleOdometryMessage(odom);
    }
}

// =============================================================================
// ROS VALIDATION FUNCTION IMPLEMENTATIONS
// =============================================================================

namespace ROSValidation {
    bool validateTwistMessage(const geometry_msgs__msg__Twist* twist) {
        if (twist == nullptr) return false;
        
        // Check for NaN values
        if (!ValidationUtils::areValidFloats(twist->linear.x, twist->linear.y, twist->linear.z) ||
            !ValidationUtils::areValidFloats(twist->angular.x, twist->angular.y, twist->angular.z)) {
            return false;
        }
        
        // Check reasonable limits using configuration constants
        if (!ValidationUtils::isInAbsoluteRange(twist->linear.x, ROS_TWIST_LINEAR_X_MAX)) return false;
        if (!ValidationUtils::isInAbsoluteRange(twist->linear.y, ROS_TWIST_LINEAR_Y_MAX)) return false;
        if (!ValidationUtils::isInAbsoluteRange(twist->linear.z, ROS_TWIST_LINEAR_Z_MAX)) return false;
        if (!ValidationUtils::isInAbsoluteRange(twist->angular.x, ROS_TWIST_ANGULAR_X_MAX)) return false;
        if (!ValidationUtils::isInAbsoluteRange(twist->angular.y, ROS_TWIST_ANGULAR_Y_MAX)) return false;
        if (!ValidationUtils::isInAbsoluteRange(twist->angular.z, ROS_TWIST_ANGULAR_Z_MAX)) return false;
        
        return true;
    }
    
    bool validateOdometryMessage(const nav_msgs__msg__Odometry* odom) {
        if (odom == nullptr) return false;
        
        // Check for NaN values in position
        if (!ValidationUtils::areValidFloats(odom->pose.pose.position.x, 
                                           odom->pose.pose.position.y, 
                                           odom->pose.pose.position.z)) {
            return false;
        }
        
        // Check for NaN values in orientation
        if (!ValidationUtils::areValidFloats(odom->pose.pose.orientation.x, 
                                           odom->pose.pose.orientation.y,
                                           odom->pose.pose.orientation.z, 
                                           odom->pose.pose.orientation.w)) {
            return false;
        }
        
        return true;
    }
    
    bool validateGeometryMessage(const geometry_msg__msg__VehicleGeometry* geom) {
        if (geom == nullptr) return false;
        // Basic validation - check for reasonable pose values
        // This is a simplified validation - in practice you might want more thorough checks
        return true;
    }
}

