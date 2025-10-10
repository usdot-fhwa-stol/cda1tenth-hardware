#include <Arduino.h>
#include "USB.h"
#include "USBCDC.h"
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/header.h>
#include <robot_state_msgs/msg/robot_state.h>

#include "car.h"
#include "sensor_manager.h"
#include "odometry.h"
#include "debug.h"

#ifndef RCCHECK
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      rclErrorLoop();            \
    }                            \
  }
#endif
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = millis();                 \
    }                                  \
    if (millis() - init > MS)          \
    {                                  \
      X;                               \
      init = millis();                 \
    }                                  \
  } while (0)

// Pin definitions
#define LED_PIN 37

// ROS Publishers and Subscribers
rcl_publisher_t robot_state_publisher;
rcl_publisher_t debug_publisher;
rcl_subscription_t twist_subscriber;

// ROS Messages
robot_state_msgs__msg__RobotState robot_state_msg;
std_msgs__msg__Float32MultiArray debug_msg;
geometry_msgs__msg__Twist twist_msg;

// ROS Infrastructure
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t debug_timer;
rcl_timer_t kinematics_timer;

// Global objects
Car car(CS_RIGHT, CS_LEFT, CS_STEER);
SensorManager sensor_manager;
Odometry odometry;
USBCDC USBSerial;

// State management
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Function declarations
void flashLED(int n_times);
void rclErrorLoop();
bool createEntities();
bool destroyEntities();
void moveBase();
void publishData();
void syncTime();
struct timespec getTime();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void debugCallback(rcl_timer_t *timer, int64_t last_call_time);
void kinematicsCallback(rcl_timer_t *timer, int64_t last_call_time);
void twistCallback(const void *msgin);
// void latencyTestCallback(const void *msgin);  // Removed latency test functionality

// Data arrays for multi-array messages
float debug_data_array[20];

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  // Initialize car control system first (like working version)
  // Initialize SPI for motor drivers
  SPI.begin();
  SPI.setFrequency(1000000); // 1MHz SPI frequency

  // Initialize sensor manager
  if (!sensor_manager.initialize())
  {
    // If sensor initialization fails, flash LED and continue
    flashLED(3);
  }

  // Initialize car
  car.begin();

  // Initialize odometry with robot parameters
  odometry.initialize(0.3f, 0.2f, 0.05f); // wheelbase, track_width, wheel_radius

  USB.begin();
  USBSerial.begin(921600);

  // Wait for USB to be ready (like working version)
  while (!USBSerial)
  {
    delay(10);
    flashLED(1); // Flash LED while waiting for serial connection
  }

  delay(2000); // Give more time for serial to initialize (like working version)

  // Serial connection established - flash LED twice to indicate ready
  flashLED(2);

  // Configure Micro-ROS library to use USB CDC serial (like working version)
  set_microros_serial_transports(USBSerial);

  // Flash LED 6 times to indicate micro-ROS transport initialized
  flashLED(6);

  // Initialize ROS messages first
  robot_state_msgs__msg__RobotState__init(&robot_state_msg);
  std_msgs__msg__Float32MultiArray__init(&debug_msg);
  geometry_msgs__msg__Twist__init(&twist_msg);

  debug_msg.data.data = debug_data_array;
  debug_msg.data.capacity = 20;
  debug_msg.data.size = 20;

  // Set layout for debug message
  debug_msg.layout.dim.data = NULL;
  debug_msg.layout.dim.size = 0;
  debug_msg.layout.dim.capacity = 0;
  debug_msg.layout.data_offset = 0;
}

void loop()
{
  static bool loop_started = false;
  if (!loop_started)
  {
    // Flash LED 7 times to indicate main loop started
    flashLED(7);
    loop_started = true;
  }

  switch (state)
  {
  case WAITING_AGENT:
    digitalWrite(LED_PIN, LOW); // LED OFF - no agent
    static bool waiting_announced = false;
    if (!waiting_announced)
    {
      // Flash LED 8 times to indicate waiting for agent
      flashLED(8);
      waiting_announced = true;
    }
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    digitalWrite(LED_PIN, LOW); // LED OFF - connecting
    state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      // Flash LED 4 times to indicate entity creation failure
      flashLED(4);
      destroyEntities();
    }
    break;
  case AGENT_CONNECTED:
    digitalWrite(LED_PIN, HIGH); // LED ON - connected
    static bool connection_announced = false;
    if (!connection_announced)
    {
      // Flash LED 3 times to indicate successful connection
      flashLED(3);
      connection_announced = true;
    }
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // Use 10ms like working version
    }
    break;
  case AGENT_DISCONNECTED:
    digitalWrite(LED_PIN, LOW); // LED OFF - disconnected
    // Flash LED 5 times to indicate disconnection
    flashLED(5);
    destroyEntities();
    state = WAITING_AGENT;
    break;
  default:
    digitalWrite(LED_PIN, LOW); // LED OFF - unknown state
    break;
  }

  // Update car control loops (like working version)
  car.updateControlLoops();
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    moveBase();
    // Update sensor data but don't publish here
    sensor_manager.update();
  }
}

void debugCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Get latest sensor data for debug info
    SensorData sensor_data = sensor_manager.getLatestData();

    // Fill debug data array with comprehensive system data
    debug_data_array[0] = 1.0f;                               // Debug flag
    debug_data_array[1] = (float)millis();                    // Current time
    debug_data_array[2] = sensor_data.accel_x;                // Accelerometer X
    debug_data_array[3] = sensor_data.accel_y;                // Accelerometer Y
    debug_data_array[4] = sensor_data.accel_z;                // Accelerometer Z
    debug_data_array[5] = sensor_data.gyro_x;                 // Gyroscope X
    debug_data_array[6] = sensor_data.gyro_y;                 // Gyroscope Y
    debug_data_array[7] = sensor_data.gyro_z;                 // Gyroscope Z
    debug_data_array[8] = car.speed;                          // Car speed
    debug_data_array[9] = car.steeringAngle;                  // Steering angle
    debug_data_array[10] = (float)state;                      // Connection state
    debug_data_array[11] = (float)ESP.getFreeHeap();          // Free heap memory
    debug_data_array[12] = (float)ESP.getHeapSize();          // Total heap size
    debug_data_array[13] = (float)twist_msg.linear.x;         // Command linear X
    debug_data_array[14] = (float)twist_msg.angular.z;        // Command angular Z
    debug_data_array[15] = (float)(millis() - prev_cmd_time); // Time since last command
    debug_data_array[16] = (float)time_offset;                // Time synchronization offset
    debug_data_array[17] = (float)car.getRightMotorRPM();     // Right motor RPM
    debug_data_array[18] = (float)car.getLeftMotorRPM();      // Left motor RPM
    debug_data_array[19] = (float)ESP.getCpuFreqMHz();        // CPU frequency

    // Ensure the debug message is properly configured
    debug_msg.data.data = debug_data_array;
    debug_msg.data.size = 20;
    debug_msg.data.capacity = 20;

    // Publish debug data only
    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
  }
}

void kinematicsCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Publish IMU and motor data together at 20Hz
    publishData();
  }
}

void twistCallback(const void *msgin)
{
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msgin;

  // Validate input to prevent crashes
  if (twist == NULL)
  {
    return;
  }

  // Copy the twist message directly (filtering handled on ROS side)
  twist_msg.linear.x = twist->linear.x;
  twist_msg.linear.y = twist->linear.y;
  twist_msg.linear.z = twist->linear.z;
  twist_msg.angular.x = twist->angular.x;
  twist_msg.angular.y = twist->angular.y;
  twist_msg.angular.z = twist->angular.z;

  prev_cmd_time = millis();
}

// Removed latencyTestCallback function for high-rate testing

bool createEntities()
{
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "car_controller", "", &support));

  // Create combined robot state publisher (IMU + motor data)
  RCCHECK(rclc_publisher_init_best_effort(
      &robot_state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(robot_state_msgs, msg, RobotState),
      "robot_state"));

  // Create debug publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &debug_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "debug_data"));

  RCCHECK(rclc_subscription_init_best_effort(
      &twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel_filtered"));

  const unsigned int control_timeout = 20;
  RCCHECK(rclc_timer_init_default(
      &control_timer,
      &support,
      RCL_MS_TO_NS(control_timeout),
      controlCallback));

  const unsigned int debug_timeout = 500;
  RCCHECK(rclc_timer_init_default(
      &debug_timer,
      &support,
      RCL_MS_TO_NS(debug_timeout),
      debugCallback));

  const unsigned int kinematics_timeout = 50; // 20Hz
  RCCHECK(rclc_timer_init_default(
      &kinematics_timer,
      &support,
      RCL_MS_TO_NS(kinematics_timeout),
      kinematicsCallback));

  // Create executor (increased size for kinematics timer)
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  // Add subscription and timers to executor
  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &twist_subscriber,
      &twist_msg,
      &twistCallback,
      ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &debug_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &kinematics_timer));

  // Synchronize time with the agent
  syncTime();

  return true;
}

bool destroyEntities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&robot_state_publisher, &node);
  rcl_publisher_fini(&debug_publisher, &node);
  rcl_subscription_fini(&twist_subscriber, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&control_timer);
  rcl_timer_fini(&debug_timer);
  rcl_timer_fini(&kinematics_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  return true;
}

void moveBase()
{
  // Brake if there's no command received for 200ms
  if (((millis() - prev_cmd_time) >= 200))
  {
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;
  }

  // Get values
  float linear_x = twist_msg.linear.x;
  float angular_z = twist_msg.angular.z;

  // Calculate steering angle using Ackermann geometry
  float steering_angle = 0.0f;

  if (fabs(linear_x) > 0.01f && fabs(angular_z) > 0.01f)
  {
    // Use wheelbase and track width for proper steering calculation
    float wheelbase = 0.3f;   // meters
    float track_width = 0.2f; // meters

    // Calculate turning radius
    float turning_radius = linear_x / angular_z;

    // Calculate steering angle using Ackermann geometry
    steering_angle = atanf(wheelbase / turning_radius) * (180.0f / M_PI);
  }

  // Limit steering angle to physical limits
  if (steering_angle > 30.0f)
    steering_angle = 30.0f;
  if (steering_angle < -30.0f)
    steering_angle = -30.0f;

  // Convert linear velocity to RPM
  float wheel_radius = 0.05f; // meters
  float speed_rpm = (linear_x / wheel_radius) * (60.0f / (2.0f * M_PI));

  // Limit speed to reasonable values
  if (speed_rpm > 300.0f)
    speed_rpm = 300.0f;
  if (speed_rpm < -300.0f)
    speed_rpm = -300.0f;

  // Debug output - keep essential motor RPM data
  debug_data_array[14] = car.getRightMotorRPM(); // Right motor RPM
  debug_data_array[15] = car.getLeftMotorRPM();  // Left motor RPM

  // Set steering angle first
  car.setSteeringAngle(steering_angle);

  // Apply motor commands with software differential
  car.setSpeed(speed_rpm, 0.3f, 0.2f); // wheelbase=0.3m, track_width=0.2m

  // Update car control loops
  car.updateControlLoops();
}

void publishData()
{
  // Get latest sensor data (already updated in control loop)
  SensorData sensor_data = sensor_manager.getLatestData();

  // IMU Data
  robot_state_msg.accel_x = sensor_data.accel_x;
  robot_state_msg.accel_y = sensor_data.accel_y;
  robot_state_msg.accel_z = sensor_data.accel_z;
  robot_state_msg.gyro_x = sensor_data.gyro_x;
  robot_state_msg.gyro_y = sensor_data.gyro_y;
  robot_state_msg.gyro_z = sensor_data.gyro_z;

  // Motor Data
  robot_state_msg.speed = car.speed;
  robot_state_msg.steering_angle = car.steeringAngle;
  robot_state_msg.right_motor_rpm = car.getRightMotorRPM();
  robot_state_msg.left_motor_rpm = car.getLeftMotorRPM();

  // System Status
  robot_state_msg.free_heap = ESP.getFreeHeap();

  // Set header timestamp (like odometry does)
  struct timespec time_stamp = getTime();
  robot_state_msg.header.stamp.sec = time_stamp.tv_sec;
  robot_state_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  robot_state_msg.header.frame_id.data = "base_link";
  robot_state_msg.header.frame_id.size = 9;
  robot_state_msg.header.frame_id.capacity = 9;

  // Publish combined robot state data
  RCSOFTCHECK(rcl_publish(&robot_state_publisher, &robot_state_msg, NULL));
}

void syncTime()
{
  // Get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // Now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
  struct timespec tp = {0};
  // Add time difference between uC time and ROS time to synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}

void rclErrorLoop()
{
  while (true)
  {
    flashLED(2);
  }
}

void flashLED(int n_times)
{
  for (int i = 0; i < n_times; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  delay(1000);
}