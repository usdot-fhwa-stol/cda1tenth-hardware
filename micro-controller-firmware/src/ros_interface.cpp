#include "ros_interface.h"
#include "car.h"
#include "sensor_manager.h"
#include "debug.h"

// Macro for executing code every N milliseconds (following micro-ROS example)
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = millis();} \
    if (millis() - init > MS) { X; init = millis();} \
} while(0)
#include <rmw_microros/rmw_microros.h>
#include <Arduino.h>

// Static member definitions
float ROSInterface::motor_data_array_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float ROSInterface::debug_data_array_[20] = {0.0f};
ROSInterface* ROSInterface::instance_ = nullptr;

// External references
extern Car car;
extern bool car_initialized;
extern struct SensorData sensorData;

ROSInterface::ROSInterface() 
    : current_state_(WAITING_AGENT)
    , last_ping_time_(0)
    , connection_failures_(0)
    , executor_timeouts_(0)
    , wheelbase_(0.185f)
    , trackWidth_(0.15f)
{
    instance_ = this;
    
    // Initialize ROS2 objects to zero
    memset(&support_, 0, sizeof(rclc_support_t));
    node_ = rcl_get_zero_initialized_node();
    executor_ = rclc_executor_get_zero_initialized_executor();
    allocator_ = rcl_get_default_allocator();
    
    // Initialize publishers
    imu_publisher_ = rcl_get_zero_initialized_publisher();
    motor_data_publisher_ = rcl_get_zero_initialized_publisher();
    debug_publisher_ = rcl_get_zero_initialized_publisher();
    
    // Initialize subscribers
    twist_subscriber_ = rcl_get_zero_initialized_subscription();
    geometry_subscriber_ = rcl_get_zero_initialized_subscription();
    
    // Initialize messages
    sensor_msgs__msg__Imu__init(&imu_msg_);
    std_msgs__msg__Float32MultiArray__init(&motor_data_msg_);
    std_msgs__msg__Float32MultiArray__init(&debug_msg_);
    geometry_msgs__msg__Twist__init(&twist_msg_);
    geometry_msg__msg__VehicleGeometry__init(&geometry_msg_);
    
    // Set up static data arrays
    motor_data_msg_.data.data = motor_data_array_;
    motor_data_msg_.data.capacity = 4;
    motor_data_msg_.data.size = 4;
    
    debug_msg_.data.data = debug_data_array_;
    debug_msg_.data.capacity = 20;
    debug_msg_.data.size = 20;
}

ROSInterface::~ROSInterface() {
    cleanup();
}

bool ROSInterface::initialize() {
    // Initialize support
    rcl_ret_t ret = rclc_support_init(&support_, 0, NULL, &allocator_);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Create node
    ret = rclc_node_init_default(&node_, "car_controller", "", &support_);
    if (ret != RCL_RET_OK) {
        rclc_support_fini(&support_);
        return false;
    }
    
    return createEntities();
}

bool ROSInterface::createEntities() {
    rcl_ret_t ret;
    
    // Create IMU publisher
    ret = rclc_publisher_init_best_effort(
        &imu_publisher_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data");
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Create motor data publisher
    ret = rclc_publisher_init_best_effort(
        &motor_data_publisher_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_data");
    if (ret != RCL_RET_OK) {
        rcl_publisher_fini(&imu_publisher_, &node_);
        return false;
    }
    
    // Create debug publisher
    ret = rclc_publisher_init_best_effort(
        &debug_publisher_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "debug_data");
    if (ret != RCL_RET_OK) {
        rcl_publisher_fini(&motor_data_publisher_, &node_);
        rcl_publisher_fini(&imu_publisher_, &node_);
        return false;
    }
    
    // Create twist subscriber
    ret = rclc_subscription_init_best_effort(
        &twist_subscriber_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
    if (ret != RCL_RET_OK) {
        rcl_publisher_fini(&debug_publisher_, &node_);
        rcl_publisher_fini(&motor_data_publisher_, &node_);
        rcl_publisher_fini(&imu_publisher_, &node_);
        return false;
    }
    
    // Create geometry subscriber
    ret = rclc_subscription_init_best_effort(
        &geometry_subscriber_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msg, msg, VehicleGeometry),
        "vehicle_geometry");
    if (ret != RCL_RET_OK) {
        rcl_subscription_fini(&twist_subscriber_, &node_);
        rcl_publisher_fini(&debug_publisher_, &node_);
        rcl_publisher_fini(&motor_data_publisher_, &node_);
        rcl_publisher_fini(&imu_publisher_, &node_);
        return false;
    }
    
    // Create executor with 2 subscriptions
    ret = rclc_executor_init(&executor_, &support_.context, 2, &allocator_);
    if (ret != RCL_RET_OK) {
        rcl_subscription_fini(&geometry_subscriber_, &node_);
        rcl_subscription_fini(&twist_subscriber_, &node_);
        rcl_publisher_fini(&debug_publisher_, &node_);
        rcl_publisher_fini(&motor_data_publisher_, &node_);
        rcl_publisher_fini(&imu_publisher_, &node_);
        return false;
    }
    
    // Add twist subscriber to executor
    ret = rclc_executor_add_subscription(&executor_, &twist_subscriber_, &twist_msg_,
        &twistCallback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        rclc_executor_fini(&executor_);
        rcl_subscription_fini(&geometry_subscriber_, &node_);
        rcl_subscription_fini(&twist_subscriber_, &node_);
        rcl_publisher_fini(&debug_publisher_, &node_);
        rcl_publisher_fini(&motor_data_publisher_, &node_);
        rcl_publisher_fini(&imu_publisher_, &node_);
        return false;
    }
    
    // Add geometry subscriber to executor
    ret = rclc_executor_add_subscription(&executor_, &geometry_subscriber_, &geometry_msg_,
        &geometryCallback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        rclc_executor_fini(&executor_);
        rcl_subscription_fini(&geometry_subscriber_, &node_);
        rcl_subscription_fini(&twist_subscriber_, &node_);
        rcl_publisher_fini(&debug_publisher_, &node_);
        rcl_publisher_fini(&motor_data_publisher_, &node_);
        rcl_publisher_fini(&imu_publisher_, &node_);
        return false;
    }
    
    return true;
}

void ROSInterface::destroyEntities() {
    if (car_initialized) {
        car.setSpeed(0.0f, wheelbase_, trackWidth_);
        car.setSteeringAngle(0.0f);
    }
    
    // Set context entity destroy session timeout to 0 for faster cleanup
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support_.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    // Clean up publishers
    rcl_publisher_fini(&imu_publisher_, &node_);
    rcl_publisher_fini(&motor_data_publisher_, &node_);
    rcl_publisher_fini(&debug_publisher_, &node_);
    
    // Clean up subscribers
    rcl_subscription_fini(&twist_subscriber_, &node_);
    rcl_subscription_fini(&geometry_subscriber_, &node_);
    
    // Clean up executor
    rclc_executor_fini(&executor_);
    
    // Clean up node and support
    rcl_node_fini(&node_);
    rclc_support_fini(&support_);
    
    // Re-initialize to clean state
    node_ = rcl_get_zero_initialized_node();
    executor_ = rclc_executor_get_zero_initialized_executor();
    imu_publisher_ = rcl_get_zero_initialized_publisher();
    motor_data_publisher_ = rcl_get_zero_initialized_publisher();
    debug_publisher_ = rcl_get_zero_initialized_publisher();
    twist_subscriber_ = rcl_get_zero_initialized_subscription();
    geometry_subscriber_ = rcl_get_zero_initialized_subscription();
}

void ROSInterface::cleanup() {
    destroyEntities();
}

bool ROSInterface::isConnected() const {
    return current_state_ == AGENT_CONNECTED;
}

void ROSInterface::update() {
    // Synchronize time with agent
    if (current_state_ == AGENT_CONNECTED) {
        rmw_uros_sync_session(1000);
    }
    
    updateConnectionState();
}

void ROSInterface::updateConnectionState() {
    switch (current_state_) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, 
                current_state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
            );
            break;
            
        case AGENT_AVAILABLE:
            current_state_ = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (current_state_ == WAITING_AGENT) {
                destroyEntities();
            }
            break;
            
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, 
                current_state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            );
            if (current_state_ == AGENT_CONNECTED) {
                // Minimal executor spin to prevent system overload
                rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));  // Minimal spin time
            }
            break;
            
        case AGENT_DISCONNECTED:
            destroyEntities();
            current_state_ = WAITING_AGENT;
            break;
            
        default:
            current_state_ = WAITING_AGENT;
            break;
    }
}


ROSInterface::ConnectionState ROSInterface::getState() const {
    return current_state_;
}

void ROSInterface::publishIMUData(const SensorData& sensorData) {
    if (!isConnected()) return;
    
    // Set IMU data
    imu_msg_.linear_acceleration.x = 0.0; // Placeholder - would need actual accel data
    imu_msg_.linear_acceleration.y = 0.0;
    imu_msg_.linear_acceleration.z = 9.81; // Gravity
    
    imu_msg_.angular_velocity.x = 0.0; // Placeholder
    imu_msg_.angular_velocity.y = 0.0;
    imu_msg_.angular_velocity.z = sensorData.gyro_z;
    
    // Set orientation (quaternion)
    imu_msg_.orientation.x = 0.0;
    imu_msg_.orientation.y = 0.0;
    imu_msg_.orientation.z = 0.0;
    imu_msg_.orientation.w = 1.0;
    
    // Set timestamp using synced time
    int64_t time_ms = rmw_uros_epoch_millis();
    if (time_ms > 0) {
        imu_msg_.header.stamp.sec = time_ms / 1000;
        imu_msg_.header.stamp.nanosec = (time_ms % 1000) * 1000000;
    } else {
        // Fallback to local time if sync fails
        uint32_t local_time_ms = millis();
        imu_msg_.header.stamp.sec = local_time_ms / 1000;
        imu_msg_.header.stamp.nanosec = (local_time_ms % 1000) * 1000000;
    }
    
    // Set frame_id
    static char frame_id[] = "imu_link";
    imu_msg_.header.frame_id.data = frame_id;
    imu_msg_.header.frame_id.size = 8;
    imu_msg_.header.frame_id.capacity = 8;
    
    rcl_publish(&imu_publisher_, &imu_msg_, NULL);
}

void ROSInterface::publishMotorData(const Car& car) {
    if (!isConnected()) return;
    
    // Update motor data array
    motor_data_array_[0] = car.getRightMotorRPMAtomic();
    motor_data_array_[1] = car.getLeftMotorRPMAtomic();
    motor_data_array_[2] = 0.0f; // Current data not available
    motor_data_array_[3] = 0.0f; // Current data not available
    
    rcl_publish(&motor_data_publisher_, &motor_data_msg_, NULL);
}

void ROSInterface::publishDebugData(const float* debugData, size_t dataSize) {
    if (!isConnected()) return;
    
    // Copy debug data to static array
    size_t copySize = min(dataSize, (size_t)20);
    for (size_t i = 0; i < copySize; i++) {
        debug_data_array_[i] = debugData[i];
    }
    
    // Update the message size
    debug_msg_.data.size = copySize;
    
    rcl_publish(&debug_publisher_, &debug_msg_, NULL);
}

void ROSInterface::setVehicleGeometry(float wheelbase, float trackWidth) {
    wheelbase_ = wheelbase;
    trackWidth_ = trackWidth;
}

// Static callback functions
void ROSInterface::twistCallback(const void* msgin) {
    if (!instance_ || !car_initialized) return;
    
    // Rate limiting to prevent message flooding
    static uint32_t last_twist_time = 0;
    uint32_t current_time = millis();
    if (current_time - last_twist_time < 50) {  // Limit to 20Hz max
        return;  // Skip this message
    }
    last_twist_time = current_time;
    
    const auto* twist = static_cast<const geometry_msgs__msg__Twist*>(msgin);
    
    // Apply safety limits
    float linear_x = twist->linear.x;
    float angular_z = twist->angular.z;
    
    // Limit values
    if (linear_x > 1.0f) linear_x = 1.0f;
    if (linear_x < -1.0f) linear_x = -1.0f;
    if (angular_z > 1.0f) angular_z = 1.0f;
    if (angular_z < -1.0f) angular_z = -1.0f;
    
    // Convert to steering angle and speed
    float steering_angle = 0.0f;
    if (angular_z != 0.0f && linear_x != 0.0f) {
        steering_angle = atanf(instance_->wheelbase_ * angular_z / linear_x) * 180.0f / M_PI;
    }
    
    // Limit steering angle
    if (steering_angle > 30.0f) steering_angle = 30.0f;
    if (steering_angle < -30.0f) steering_angle = -30.0f;
    
    // Convert speed to RPM
    float speed_rpm = linear_x * 4.0f * 60.0f / (M_PI * 0.06f); // Scale factor
    
    // TEMPORARILY DISABLED - Test if car control is causing freeze
    // car.setSteeringAngle(steering_angle);
    // car.setSpeed(speed_rpm, instance_->wheelbase_, instance_->trackWidth_);
}

void ROSInterface::geometryCallback(const void* msgin) {
    if (!instance_) return;
    
    const auto* geom = static_cast<const geometry_msg__msg__VehicleGeometry*>(msgin);
    
    // Calculate wheelbase and track width from geometry
    float x_front = 0.5f * (geom->front_left_wheel_pose.position.x + geom->front_right_wheel_pose.position.x);
    float x_back = 0.5f * (geom->back_left_wheel_pose.position.x + geom->back_right_wheel_pose.position.x);
    float wheelbase = fabsf(x_front - x_back);
    
    float track_width = fabsf(geom->front_left_wheel_pose.position.y - geom->front_right_wheel_pose.position.y);
    
    instance_->setVehicleGeometry(wheelbase, track_width);
}


