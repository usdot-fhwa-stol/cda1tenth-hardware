#include <Arduino.h>
#include "USB.h"
#include "USBCDC.h"
// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <micro_ros_platformio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msg/msg/vehicle_geometry.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

// Include car control logic
#include "car.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rcl_timer_t motor_rpm_timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
rcl_subscription_t twist_subscriber;
std_msgs__msg__Int32 msg;

// Motor RPM publisher
rcl_publisher_t motor_rpm_publisher;
std_msgs__msg__Float32MultiArray motor_rpm_msg;

// Additional ROS2 objects
rcl_subscription_t subscriber;
rcl_subscription_t geom_subscriber;
std_msgs__msg__Int32 sub_msg;
geometry_msg__msg__VehicleGeometry geom_msg;
geometry_msgs__msg__Twist twist_msg;

float g_wheelbase = 0.185f;
float g_track_width = 0.15f; 
float g_steering_scaling_factor = 1.5f; // Scaling factor for encoder angle to wheel steering angle
float o_speed_scaling_factor = 1.5f; // Factor to scale the speed command

// Car control instance
Car car(CS_RIGHT, CS_LEFT, CS_STEER);

// Car initialization flags
bool car_initialized = false;
TaskHandle_t updateTaskHandle = NULL;
TaskHandle_t microRosTaskHandle = NULL;

// Declare USBSerial object
USBCDC USBSerial;

typedef enum {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} agent_state_t;

static volatile agent_state_t state = WAITING_AGENT; // Shared between tasks
static SemaphoreHandle_t stateMutex;

typedef struct {
  TickType_t last_wake_time;
  TickType_t interval_ticks;
} periodic_task_t;

void motor_rpm_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL && car_initialized) {
    // Get motor RPMs
    float right_rpm = car.getRightMotorRPM();
    float left_rpm = car.getLeftMotorRPM();
    float steering_angle = car.steeringMotor.getSteeringAngle();
    
    // Set up the message
    motor_rpm_msg.data.size = 3;
    motor_rpm_msg.data.data[0] = right_rpm;
    motor_rpm_msg.data.data[1] = left_rpm;
    motor_rpm_msg.data.data[2] = steering_angle; // Add steering angle to the message
    
    // Publish motor RPM data
    rcl_publish(&motor_rpm_publisher, &motor_rpm_msg, NULL);
    
  }
}

void twist_callback(const void * msgin) {
  const auto * twist = static_cast<const geometry_msgs__msg__Twist *>(msgin);
  float steering_angle_rad = twist->angular.z * g_steering_scaling_factor; // Scale the steering angle

  float max_steering_angle = 0.45f * g_steering_scaling_factor; // ~30 degrees in radians

  // Apply steering limits (assuming max 30 degrees each way)
  if (steering_angle_rad > max_steering_angle) steering_angle_rad = max_steering_angle;
  if (steering_angle_rad < -max_steering_angle) steering_angle_rad = -max_steering_angle;
  
  // Convert steering angle from radians to degrees
  float steering_angle_deg = steering_angle_rad * 180.0f / M_PI;
  
  // Convert speed from m/s to RPM (approximate conversion)
  // Assuming wheel diameter of 0.06m and gear ratio
  float speed_rpm = twist->linear.x * o_speed_scaling_factor * 60.0f / (M_PI * 0.06f);

  // Apply speed limits
  if (speed_rpm > 500.0f) speed_rpm = 500.0f;
  if (speed_rpm < -500.0f) speed_rpm = -500.0f;
  
  // Update car control
  car.setSteeringAngle(steering_angle_deg);
  car.setSpeed(speed_rpm, g_wheelbase, g_track_width);
  
  // geometry_msgs__msg__Twist out;
  // geometry_msgs__msg__Twist__init(&out);
  // out.linear.x  = twist->linear.x;
  // out.linear.y  = twist->linear.y;
  // out.linear.z  = twist->linear.z;
  // out.angular.x = twist->angular.x;
  // out.angular.y = twist->angular.y;
  // out.angular.z = twist->angular.z;
  // rcl_publish(&twist_publisher, &out, nullptr);
}

// Callback function for the vehicle geometry subscriber
void vehicle_geometry_callback(const void * msgin)
{
  // Cast the raw pointer to the generated struct
  auto * geom_msg = static_cast<const geometry_msg__msg__VehicleGeometry *>(msgin);

  // Pointers to each Pose field
  const geometry_msgs__msg__Pose * poses[] = {
    &geom_msg->front_left_wheel_pose,
    &geom_msg->front_right_wheel_pose,
    &geom_msg->back_left_wheel_pose,
    &geom_msg->back_right_wheel_pose,
    &geom_msg->lidar_pose,
    &geom_msg->camera_pose,
    &geom_msg->imu_pose
  };

  // Human‑readable labels
  const char * labels[] = {
    "Front-Left Wheel",
    "Front-Right Wheel",
    "Back-Left Wheel",
    "Back-Right Wheel",
    "LiDAR",
    "Camera",
    "IMU"
  };

  // Throttle logging to avoid blocking the executor/USB CDC
  static uint32_t last_log_ms = 0;
  const uint32_t now_ms = uxr_millis();
  if (now_ms - last_log_ms >= 1000) {
    constexpr size_t N = sizeof(poses) / sizeof(poses[0]);
    for (size_t i = 0; i < N; ++i) {
      const auto & p = *poses[i];
      USBSerial.printf(
        "%s — pos:(%.2f, %.2f, %.2f) ori:(%.2f, %.2f, %.2f, %.2f)\n",
        labels[i],
        p.position.x,
        p.position.y,
        p.position.z,
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w
      );
    }
    last_log_ms = now_ms;
  }

  float x_front_axle = 0.5f * (geom_msg->front_left_wheel_pose.position.x + geom_msg->front_right_wheel_pose.position.x);
  float x_back_axle  = 0.5f * (geom_msg->back_left_wheel_pose.position.x  + geom_msg->back_right_wheel_pose.position.x);
  g_wheelbase = fabsf(x_front_axle - x_back_axle);

  g_track_width = fabsf(geom_msg->front_left_wheel_pose.position.y - geom_msg->front_right_wheel_pose.position.y);
}


void initializeCar() {
  if (!car_initialized) {    
    // Initialize SPI for motor drivers
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
    delay(100);
    
    // Initialize car control
    car.begin();
    delay(100);

    car.setSpeed(0.0f, g_wheelbase, g_track_width); // Start with zero speed
    car.setSteeringAngle(0.0f); // Start with zero steering angle
    
    car_initialized = true;
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "car_controller", "", &support));

  // create motor RPM publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &motor_rpm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "motor_rpms"));
  
  // create subscriber for vehicle geometry
  RCCHECK(rclc_subscription_init_best_effort(
    &geom_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msg, msg, VehicleGeometry),
    "vehicle_geometry"));

  // create subscriber for twist messages (cmd_vel)
  RCCHECK(rclc_subscription_init_best_effort(
    &twist_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create motor RPM timer (publish every 100ms)
  const unsigned int motor_rpm_timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &motor_rpm_timer,
    &support,
    RCL_MS_TO_NS(motor_rpm_timer_timeout),
    motor_rpm_timer_callback));


  // num handles = total_of_subscribers + timers (publisher is not counted)
  unsigned int number_of_handles = 3;
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &motor_rpm_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &geom_subscriber, &geom_msg,
    &vehicle_geometry_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &twist_subscriber, &twist_msg,
    &twist_callback, ON_NEW_DATA));
  // Ensure spin_some returns promptly to keep the loop responsive
  rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(2));
  
  // Initialize the car control system
  initializeCar();

  return true;
}

void destroy_entities()
{
  if (car_initialized) {
    car.setSpeed(0.0f, g_wheelbase, g_track_width); // Start with zero speed
    car.setSteeringAngle(0.0f); // Start with zero steering angle
    car_initialized = false;
  }

  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&motor_rpm_publisher, &node);
  rcl_subscription_fini(&geom_subscriber, &node);
  rcl_subscription_fini(&twist_subscriber, &node);
  rcl_timer_fini(&motor_rpm_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  // Initialize USB CDC
  USB.begin();
  USBSerial.begin(921600);
  
  // Wait for USB to be ready
  while (!USBSerial) {
    delay(10);
  }
  
  delay(2000); // Give more time for serial to initialize
  
  // Configure Micro-ROS library to use USB CDC serial
  set_microros_serial_transports(USBSerial);

  state = WAITING_AGENT;  
  msg.data = 0;
  
  // Initialize motor RPM message
  motor_rpm_msg.data.size = 3;
  motor_rpm_msg.data.capacity = 3;
  motor_rpm_msg.data.data = (float*)malloc(3 * sizeof(float));
}

// float angle = 25.00f;
// uint32_t lastApplyMicros = micros();

// void testMotorControl() {
//   initializeCar();

//   float steering_angle = car.steeringMotor.getSteeringAngle();

//   // Set initial speed and angle
//   car.setSpeed(200.0f, g_wheelbase, g_track_width);

//   const uint32_t now = micros();
//   const bool timeElapsed = (now - lastApplyMicros) / 1000 >= 2000;

//   if (!timeElapsed) {
//     return;
//   }
//   lastApplyMicros = now;
//   USBSerial.println("Time Elapsed");
//   USBSerial.println(steering_angle);

//   car.setSteeringAngle(angle);
//   angle = -angle;
//   USBSerial.println(angle);
// }


void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
  

  // testMotorControl();
  if (car_initialized) {
    car.updateControlLoops();
  }
}