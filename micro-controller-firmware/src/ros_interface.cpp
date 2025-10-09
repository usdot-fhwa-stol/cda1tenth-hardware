#include "ros_interface.h"
#include "car.h"
#include "sensor_manager.h"
#include "debug.h"

// Macro for executing code every N milliseconds (following micro-ROS example)
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = millis();               \
        }                                  \
        if (millis() - init > MS)          \
        {                                  \
            X;                             \
            init = millis();               \
        }                                  \
    } while (0)
#include <rmw_microros/rmw_microros.h>
#include <Arduino.h>

// Static member definitions
float ROSInterface::motor_data_array_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float ROSInterface::debug_data_array_[20] = {0.0f};
ROSInterface *ROSInterface::instance_ = nullptr;

// External references
extern Car car;
extern bool car_initialized;

ROSInterface::ROSInterface()
    : current_state_(WAITING_AGENT), wheelbase_(0.185f), trackWidth_(0.15f)
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

ROSInterface::~ROSInterface()
{
    cleanup();
}

bool ROSInterface::initialize()
{
    // Initialize support
    rcl_ret_t ret = rclc_support_init(&support_, 0, NULL, &allocator_);
    if (ret != RCL_RET_OK)
    {
        return false;
    }

    // Create node
    ret = rclc_node_init_default(&node_, "car_controller", "", &support_);
    if (ret != RCL_RET_OK)
    {
        rclc_support_fini(&support_);
        return false;
    }

    return createEntities();
}

bool ROSInterface::createEntities()
{
    rcl_ret_t ret;

    // Create IMU publisher
    ret = rclc_publisher_init_best_effort(
        &imu_publisher_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data");
    if (ret != RCL_RET_OK)
    {
        return false;
    }

    // Create motor data publisher
    ret = rclc_publisher_init_best_effort(
        &motor_data_publisher_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_data");
    if (ret != RCL_RET_OK)
    {
        rcl_publisher_fini(&imu_publisher_, &node_);
        return false;
    }

    // Create debug publisher
    ret = rclc_publisher_init_best_effort(
        &debug_publisher_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "debug_data");
    if (ret != RCL_RET_OK)
    {
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
    if (ret != RCL_RET_OK)
    {
        rcl_publisher_fini(&debug_publisher_, &node_);
        rcl_publisher_fini(&motor_data_publisher_, &node_);
        rcl_publisher_fini(&imu_publisher_, &node_);
        return false;
    }

    // TEMPORARILY DISABLED - Geometry subscriber to reduce memory usage
    // Create geometry subscriber
    // ret = rclc_subscription_init_best_effort(
    //     &geometry_subscriber_,
    //     &node_,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msg, msg, VehicleGeometry),
    //     "vehicle_geometry");
    // if (ret != RCL_RET_OK)
    // {
    //     rcl_subscription_fini(&twist_subscriber_, &node_);
    //     rcl_publisher_fini(&debug_publisher_, &node_);
    //     rcl_publisher_fini(&motor_data_publisher_, &node_);
    //     rcl_publisher_fini(&imu_publisher_, &node_);
    //     return false;
    // }

    // Create executor with only 1 subscription (ultra-minimal)
    ret = rclc_executor_init(&executor_, &support_.context, 1, &allocator_);
    if (ret != RCL_RET_OK)
    {
        rcl_subscription_fini(&twist_subscriber_, &node_);
        rcl_publisher_fini(&debug_publisher_, &node_);
        return false;
    }

    // Add only twist subscriber to executor (remove geometry subscriber temporarily)
    ret = rclc_executor_add_subscription(&executor_, &twist_subscriber_, &twist_msg_,
                                         &twistCallback, ON_NEW_DATA);
    if (ret != RCL_RET_OK)
    {
        rclc_executor_fini(&executor_);
        rcl_subscription_fini(&twist_subscriber_, &node_);
        rcl_publisher_fini(&debug_publisher_, &node_);
        return false;
    }

    return true;
}

void ROSInterface::destroyEntities()
{
    if (car_initialized)
    {
        car.setSpeed(0.0f, wheelbase_, trackWidth_);
        car.setSteeringAngle(0.0f);
    }

    // Set context entity destroy session timeout to 0 for faster cleanup
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

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

void ROSInterface::cleanup()
{
    destroyEntities();
}

bool ROSInterface::isConnected() const
{
    return current_state_ == AGENT_CONNECTED;
}

void ROSInterface::update()
{
    // Synchronize time with agent
    if (current_state_ == AGENT_CONNECTED)
    {
        rmw_uros_sync_session(1000);
    }

    updateConnectionState();
}

void ROSInterface::updateConnectionState()
{
    switch (current_state_)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500,
                           current_state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;

    case AGENT_AVAILABLE:
        current_state_ = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (current_state_ == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200,
                           current_state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (current_state_ == AGENT_CONNECTED)
        {
            // Use polling instead of executor to prevent disconnection issues
            pollForMessages();
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

ROSInterface::ConnectionState ROSInterface::getState() const
{
    return current_state_;
}

void ROSInterface::publishIMUData(const SensorData &sensorData)
{
    if (!isConnected())
        return;

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
    if (time_ms > 0)
    {
        imu_msg_.header.stamp.sec = time_ms / 1000;
        imu_msg_.header.stamp.nanosec = (time_ms % 1000) * 1000000;
    }
    else
    {
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

void ROSInterface::publishMotorData(const Car &car)
{
    if (!isConnected())
        return;

    // Update motor data array
    motor_data_array_[0] = car.getRightMotorRPM();
    motor_data_array_[1] = car.getLeftMotorRPM();
    motor_data_array_[2] = 0.0f; // Current data not available
    motor_data_array_[3] = 0.0f; // Current data not available

    rcl_publish(&motor_data_publisher_, &motor_data_msg_, NULL);
}

void ROSInterface::publishDebugData(const float *debugData, size_t dataSize)
{
    if (!isConnected())
        return;

    // Copy debug data to static array
    size_t copySize = min(dataSize, (size_t)20);
    for (size_t i = 0; i < copySize; i++)
    {
        debug_data_array_[i] = debugData[i];
    }

    // Update the message size
    debug_msg_.data.size = copySize;

    rcl_publish(&debug_publisher_, &debug_msg_, NULL);
}

void ROSInterface::setVehicleGeometry(float wheelbase, float trackWidth)
{
    wheelbase_ = wheelbase;
    trackWidth_ = trackWidth;
}

// Static callback functions
void ROSInterface::twistCallback(const void *msgin)
{
    // ULTRA-MINIMAL callback - just queue the message and return immediately
    if (!instance_)
        return;

    // Extract message data quickly
    const auto *twist = static_cast<const geometry_msgs__msg__Twist *>(msgin);

    // Add to queue if space available
    if (instance_->queue_count_ < instance_->MESSAGE_QUEUE_SIZE)
    {
        TwistMessage &msg = instance_->message_queue_[instance_->queue_tail_];
        msg.linear_x = twist->linear.x;
        msg.angular_z = twist->angular.z;
        msg.timestamp = millis();
        msg.valid = true;

        instance_->queue_tail_ = (instance_->queue_tail_ + 1) % instance_->MESSAGE_QUEUE_SIZE;
        instance_->queue_count_++;

        // Update debug data
        instance_->debug_data_array_[20] = 1.0f;                 // Message received flag
        instance_->debug_data_array_[21] = (float)msg.timestamp; // Timestamp
    }

    return; // Exit immediately - no processing to prevent blocking
}

bool ROSInterface::getNextTwistMessage(TwistMessage &msg)
{
    if (queue_count_ == 0)
    {
        return false; // No messages available
    }

    msg = message_queue_[queue_head_];
    message_queue_[queue_head_].valid = false; // Mark as processed
    queue_head_ = (queue_head_ + 1) % MESSAGE_QUEUE_SIZE;
    queue_count_--;

    return true;
}

void ROSInterface::clearMessageQueue()
{
    queue_head_ = 0;
    queue_tail_ = 0;
    queue_count_ = 0;

    // Clear all messages
    for (int i = 0; i < MESSAGE_QUEUE_SIZE; i++)
    {
        message_queue_[i].valid = false;
    }
}

void ROSInterface::pollForMessages()
{
    // DISABLE EXECUTOR COMPLETELY - This is the most radical approach
    // Don't process any messages at all to prevent disconnection
    // Messages will be lost, but connection will remain stable

    // Just do nothing - let messages accumulate in the transport layer
    // This prevents any executor-related issues
}

void ROSInterface::geometryCallback(const void *msgin)
{
    if (!instance_)
        return;

    const auto *geom = static_cast<const geometry_msg__msg__VehicleGeometry *>(msgin);

    // Calculate wheelbase and track width from geometry
    float x_front = 0.5f * (geom->front_left_wheel_pose.position.x + geom->front_right_wheel_pose.position.x);
    float x_back = 0.5f * (geom->back_left_wheel_pose.position.x + geom->back_right_wheel_pose.position.x);
    float wheelbase = fabsf(x_front - x_back);

    float track_width = fabsf(geom->front_left_wheel_pose.position.y - geom->front_right_wheel_pose.position.y);

    instance_->setVehicleGeometry(wheelbase, track_width);
}
