#include <Arduino.h>
#include "USB.h"
#include "USBCDC.h"
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <string.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/header.h>
#include <car_state_msg/msg/car_state.h>
#include <car_config_msg/msg/car_config.h>

#include "car.h"
#include "sensor_manager.h"
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

// Default parameter values
#define DEFAULT_ENCODER_OFFSET 187.5f
#define DEFAULT_WHEELBASE 0.185f
#define DEFAULT_TRACK_WIDTH 0.15f

// Control constants
#define COMMAND_TIMEOUT_MS 200
#define MIN_VELOCITY_THRESHOLD 0.01f
#define MAX_STEERING_ANGLE_DEG 30.0f
#define WHEEL_RADIUS_M 0.0325f
#define MAX_RPM 300.0f
#define DEBUG_ARRAY_RIGHT_MOTOR_RPM_IDX 14
#define DEBUG_ARRAY_LEFT_MOTOR_RPM_IDX 15

// ROS Publishers and Subscribers
rcl_publisher_t car_state_publisher;
rcl_publisher_t debug_publisher;
rcl_subscription_t twist_subscriber;
rcl_subscription_t car_config_subscriber;

// ROS Messages
car_state_msg__msg__CarState car_state_msg;
std_msgs__msg__Float32MultiArray debug_msg;
geometry_msgs__msg__Twist twist_msg;
car_config_msg__msg__CarConfig car_config_msg;

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
USBCDC USBSerial;

// State management
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

// Runtime parameters (updated from car_config message)
float max_steering_angle = MAX_STEERING_ANGLE_DEG;
float max_rpm = MAX_RPM;
float wheel_radius = WHEEL_RADIUS_M;

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
void carConfigCallback(const void *msgin);

// Data arrays for multi-array messages
float debug_data_array[20];

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  SPI.begin();
  SPI.setFrequency(1000000);

  if (!sensor_manager.initialize())
  {
    flashLED(3);
  }

  car.begin();
  USB.begin();
  USBSerial.begin(921600);

  while (!USBSerial)
  {
    delay(10);
    flashLED(1);
  }

  delay(2000);
  flashLED(2);

  set_microros_serial_transports(USBSerial);
  flashLED(6);

  car_state_msg__msg__CarState__init(&car_state_msg);
  std_msgs__msg__Float32MultiArray__init(&debug_msg);
  geometry_msgs__msg__Twist__init(&twist_msg);
  car_config_msg__msg__CarConfig__init(&car_config_msg);

  debug_msg.data.data = debug_data_array;
  debug_msg.data.capacity = 20;
  debug_msg.data.size = 20;

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
    flashLED(7);
    loop_started = true;
  }

  switch (state)
  {
  case WAITING_AGENT:
    digitalWrite(LED_PIN, LOW);
    static bool waiting_announced = false;
    if (!waiting_announced)
    {
      flashLED(8);
      waiting_announced = true;
    }
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    digitalWrite(LED_PIN, LOW);
    state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      flashLED(4);
      destroyEntities();
    }
    break;
  case AGENT_CONNECTED:
    digitalWrite(LED_PIN, HIGH);
    static bool connection_announced = false;
    if (!connection_announced)
    {
      flashLED(3);
      connection_announced = true;
    }
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    break;
  case AGENT_DISCONNECTED:
    digitalWrite(LED_PIN, LOW);
    flashLED(5);
    destroyEntities();
    state = WAITING_AGENT;
    break;
  default:
    digitalWrite(LED_PIN, LOW);
    break;
  }

  car.updateControlLoops();
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    moveBase();
    sensor_manager.update();
  }
}

void debugCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    SensorData sensor_data = sensor_manager.getLatestData();

    debug_data_array[0] = 1.0f;
    debug_data_array[1] = (float)millis();
    debug_data_array[2] = sensor_data.accel_x;
    debug_data_array[3] = sensor_data.accel_y;
    debug_data_array[4] = sensor_data.accel_z;
    debug_data_array[5] = sensor_data.gyro_x;
    debug_data_array[6] = sensor_data.gyro_y;
    debug_data_array[7] = sensor_data.gyro_z;
    debug_data_array[8] = car.speed;
    debug_data_array[9] = car.steeringAngle;
    debug_data_array[10] = (float)state;
    debug_data_array[11] = (float)ESP.getFreeHeap();
    debug_data_array[12] = (float)ESP.getHeapSize();
    debug_data_array[13] = (float)twist_msg.linear.x;
    debug_data_array[14] = (float)twist_msg.angular.z;
    debug_data_array[15] = (float)(millis() - prev_cmd_time);
    debug_data_array[16] = (float)time_offset;
    debug_data_array[17] = (float)car.getRightMotorRPM();
    debug_data_array[18] = (float)car.getLeftMotorRPM();
    debug_data_array[19] = (float)ESP.getCpuFreqMHz();

    debug_msg.data.data = debug_data_array;
    debug_msg.data.size = 20;
    debug_msg.data.capacity = 20;

    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
  }
}

void kinematicsCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    publishData();
  }
}

void twistCallback(const void *msgin)
{
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msgin;

  if (twist == NULL)
  {
    return;
  }

  twist_msg.linear.x = twist->linear.x;
  twist_msg.linear.y = twist->linear.y;
  twist_msg.linear.z = twist->linear.z;
  twist_msg.angular.x = twist->angular.x;
  twist_msg.angular.y = twist->angular.y;
  twist_msg.angular.z = twist->angular.z;

  prev_cmd_time = millis();
}

bool createEntities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "car_controller", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(
      &car_state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(car_state_msg, msg, CarState),
      "car/car_state"));

  RCCHECK(rclc_publisher_init_best_effort(
      &debug_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "car/debug_data"));

  RCCHECK(rclc_subscription_init_best_effort(
      &twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel_filtered"));

  RCCHECK(rclc_subscription_init_best_effort(
      &car_config_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(car_config_msg, msg, CarConfig),
      "car/config"));

  car.steeringMotor.setEncoderOffset(DEFAULT_ENCODER_OFFSET);
  car.wheelbase = DEFAULT_WHEELBASE;
  car.trackWidth = DEFAULT_TRACK_WIDTH;

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

  const unsigned int kinematics_timeout = 50;
  RCCHECK(rclc_timer_init_default(
      &kinematics_timer,
      &support,
      RCL_MS_TO_NS(kinematics_timeout),
      kinematicsCallback));

  executor = rclc_executor_get_zero_initialized_executor();
  // 2 subscriptions + 3 timers = 5 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &twist_subscriber,
      &twist_msg,
      &twistCallback,
      ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &car_config_subscriber,
      &car_config_msg,
      &carConfigCallback,
      ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &debug_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &kinematics_timer));

  syncTime();

  return true;
}

bool destroyEntities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t rc = rcl_publisher_fini(&car_state_publisher, &node);
  rc += rcl_publisher_fini(&debug_publisher, &node);
  rc += rcl_subscription_fini(&twist_subscriber, &node);
  rc += rcl_subscription_fini(&car_config_subscriber, &node);
  rc += rcl_node_fini(&node);
  rc += rcl_timer_fini(&control_timer);
  rc += rcl_timer_fini(&debug_timer);
  rc += rcl_timer_fini(&kinematics_timer);
  rc += rclc_executor_fini(&executor);
  rc += rclc_support_fini(&support);

  return rc == RCL_RET_OK;
}

void moveBase()
{
  if (((millis() - prev_cmd_time) >= COMMAND_TIMEOUT_MS))
  {
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;
  }

  float linear_x = twist_msg.linear.x;
  float angular_z = twist_msg.angular.z;
  float steering_angle = 0.0f;

  if (fabs(linear_x) > MIN_VELOCITY_THRESHOLD && fabs(angular_z) > MIN_VELOCITY_THRESHOLD)
  {
    float turning_radius = linear_x / angular_z;
    steering_angle = atanf(car.wheelbase / turning_radius) * (180.0f / M_PI);
  }

  if (steering_angle > max_steering_angle)
    steering_angle = max_steering_angle;
  if (steering_angle < -max_steering_angle)
    steering_angle = -max_steering_angle;

  float speed_rpm = (linear_x / wheel_radius) * (60.0f / (2.0f * M_PI));

  if (speed_rpm > max_rpm)
    speed_rpm = max_rpm;
  if (speed_rpm < -max_rpm)
    speed_rpm = -max_rpm;

  debug_data_array[DEBUG_ARRAY_RIGHT_MOTOR_RPM_IDX] = car.getRightMotorRPM();
  debug_data_array[DEBUG_ARRAY_LEFT_MOTOR_RPM_IDX] = car.getLeftMotorRPM();

  car.setSteeringAngle(steering_angle);
  car.setSpeed(speed_rpm, car.wheelbase, car.trackWidth);
  car.updateControlLoops();
}

void publishData()
{
  SensorData sensor_data = sensor_manager.getLatestData();

  car_state_msg.accel_x = sensor_data.accel_x;
  car_state_msg.accel_y = sensor_data.accel_y;
  car_state_msg.accel_z = sensor_data.accel_z;
  car_state_msg.gyro_x = sensor_data.gyro_x;
  car_state_msg.gyro_y = sensor_data.gyro_y;
  car_state_msg.gyro_z = sensor_data.gyro_z;
  car_state_msg.speed = car.speed;
  car_state_msg.steering_angle = car.getActualSteeringAngle();
  car_state_msg.right_motor_rpm = car.getRightMotorRPM();
  car_state_msg.left_motor_rpm = car.getLeftMotorRPM();

  struct timespec time_stamp = getTime();
  car_state_msg.header.stamp.sec = time_stamp.tv_sec;
  car_state_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  car_state_msg.header.frame_id.data = "base_link";
  car_state_msg.header.frame_id.size = 9;
  car_state_msg.header.frame_id.capacity = 9;

  RCSOFTCHECK(rcl_publish(&car_state_publisher, &car_state_msg, NULL));
}

void carConfigCallback(const void *msgin)
{
  const car_config_msg__msg__CarConfig *config = (const car_config_msg__msg__CarConfig *)msgin;

  if (config == NULL)
  {
    return;
  }

  // Update car configuration from received message
  car.wheelbase = (float)config->wheelbase;
  car.trackWidth = (float)config->track_width;
  wheel_radius = (float)config->wheel_radius;
  car.steeringMotor.setEncoderOffset((float)config->encoder_offset);
  max_steering_angle = (float)config->max_steering_angle;
  max_rpm = (float)config->max_rpm;
}

void syncTime()
{
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
  struct timespec tp = {0};
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