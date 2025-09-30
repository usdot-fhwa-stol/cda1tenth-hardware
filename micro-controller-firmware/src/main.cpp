#include <Arduino.h>
#include "USB.h"
#include "USBCDC.h"
#include "SparkFunLSM6DSO.h"
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
// #include <rclc_parameter/rclc_parameter.h>
 
#include <geometry_msg/msg/vehicle_geometry.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <math.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>
 
// Include car control logic
#include "car.h"
 
#define CS_IMU      14
#define LED_PIN     37
 
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
rcl_subscription_t twist_subscriber;
 
// Motor RPM publisher
rcl_publisher_t motor_rpm_publisher;
std_msgs__msg__Float32MultiArray motor_rpm_msg;
 
// Additional ROS2 objects
rcl_subscription_t geom_subscriber;
geometry_msg__msg__VehicleGeometry geom_msg;
geometry_msgs__msg__Twist twist_msg;

// rclc_parameter_server_t param_server;
// static double param_wheelbase = 0.185;
unsigned int g_odom_period_ms = 100; // 10Hz odometry for maximum stability
float g_wheel_radius = 0.03;

extern bool odometry_init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);
extern void odometry_fini(rcl_node_t* node);
extern void odometry_reset(float x, float y, float yaw_rad);
extern void odometry_set_period_ms(unsigned int period_ms);
extern void odometry_set_wheel_radius(float r);
 
static float g_odom_x = 0.f, g_odom_y = 0.f, g_odom_yaw = 0.f;
 
float g_wheelbase = 0.185f;
float g_track_width = 0.15f;
 
// Desmos link: https://www.desmos.com/calculator/hzw38ukeor
float g_steering_scaling_factor = (1.0f/0.87f); // Scaling factor for encoder angle to wheel steering angle
float o_speed_scaling_factor = 4.0f; // Factor to scale the speed command
 
// Steering control constants
const float MAX_STEERING_ANGLE = 45.0f; // degrees
 
// Car control instance
Car car(CS_RIGHT, CS_LEFT, CS_STEER);
 
// Car initialization flags
bool car_initialized = false;
TaskHandle_t updateTaskHandle = NULL;
TaskHandle_t microRosTaskHandle = NULL;
 
// IMU instance
LSM6DSO IMU;
 
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

// LED status variables
static uint32_t last_led_update = 0;
static uint32_t led_blink_count = 0;
 
typedef struct {
  TickType_t last_wake_time;
  TickType_t interval_ticks;
} periodic_task_t;

// LED status update function
void updateLEDStatus() {
  // For testing - keep LED ON by default
  digitalWrite(LED_PIN, HIGH);
  
  // Original status code (commented out for testing)
  /*
  uint32_t now = millis();
  if (now - last_led_update < 100) return; // Update every 100ms
  last_led_update = now;
  
  switch (state) {
    case WAITING_AGENT:
      // Slow blink (1 second on, 1 second off)
      digitalWrite(LED_PIN, (led_blink_count / 10) % 2);
      break;
    case AGENT_AVAILABLE:
      // Fast blink (200ms on, 200ms off)
      digitalWrite(LED_PIN, (led_blink_count / 2) % 2);
      break;
    case AGENT_CONNECTED:
      // Solid on
      digitalWrite(LED_PIN, HIGH);
      break;
    case AGENT_DISCONNECTED:
      // Double blink pattern
      digitalWrite(LED_PIN, (led_blink_count / 5) % 4 < 2);
      break;
    default:
      digitalWrite(LED_PIN, LOW);
      break;
  }
  led_blink_count++;
  */
}
 
// Motor RPM timer callback disabled for stability
// void motor_rpm_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
//   (void) last_call_time;
//   if (timer != NULL && car_initialized) {
//     // Use cached values from car control task to avoid blocking SPI
//     static float cached_right_rpm = 0.0f;
//     static float cached_left_rpm = 0.0f;
//     static float cached_steering_angle = 0.0f;
//     static uint32_t last_update = 0;
//     
//     // Only update cache every 50ms to reduce SPI blocking
//     uint32_t now = millis();
//     if (now - last_update > 50) {
//       cached_right_rpm = car.getRightMotorRPM();
//       cached_left_rpm = car.getLeftMotorRPM();
//       cached_steering_angle = car.steeringMotor.getSteeringAngle();
//       last_update = now;
//     }
//    
//     // Set up the message
//     motor_rpm_msg.data.size = 3;
//     motor_rpm_msg.data.data[0] = cached_right_rpm;
//     motor_rpm_msg.data.data[1] = cached_left_rpm;
//     motor_rpm_msg.data.data[2] = cached_steering_angle;
//    
//     // Publish motor RPM data
//     rcl_publish(&motor_rpm_publisher, &motor_rpm_msg, NULL);
//   }
// }
 
float getSteeringAngle(float omega, float vel) {
  if (omega == 0.0f || vel == 0.0f) {
    return 0.0f;
  }
  return atanf(g_wheelbase * omega / vel) * 180.0f / M_PI;
}
 
void twist_callback(const void * msgin) {
  if (!car_initialized) return; // Don't process if car not ready
  
  const auto * twist = static_cast<const geometry_msgs__msg__Twist *>(msgin);
 
  // Use bicycle model for proper steering angle calculation
  float steering_angle_deg = getSteeringAngle(twist->angular.z, twist->linear.x);
  float speed_rpm = twist->linear.x * o_speed_scaling_factor * 60.0f / (M_PI * 0.06f);
 
  // Update car control without mutex to avoid blocking
  // This is safe since we're not running control loops
  car.setSteeringAngle(steering_angle_deg);
  car.setSpeed(speed_rpm, g_wheelbase, g_track_width);
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
      // USBSerial.printf(
      //   "%s — pos:(%.2f, %.2f, %.2f) ori:(%.2f, %.2f, %.2f, %.2f)\n",
      //   labels[i],
      //   p.position.x,
      //   p.position.y,
      //   p.position.z,
      //   p.orientation.x,
      //   p.orientation.y,
      //   p.orientation.z,
      //   p.orientation.w
      // );
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
   
    // Initialize IMU - Use these calls to return values IMU.readFloatAccelX(), IMU.readFloatAccelY(), IMU.readFloatAccelZ(), IMU.readFloatGyroX(), IMU.readFloatGyroY(), IMU.readFloatGyroZ(), IMU.readTempF()
    IMU.beginSPI(CS_IMU);
    IMU.initialize(BASIC_SETTINGS);
 
    // Initialize car control
    car.begin();
    delay(100);
 
    car.setSpeed(0.0f, g_wheelbase, g_track_width); // Start with zero speed
    car.setSteeringAngle(0.0f); // Start with zero steering angle
   
    car_initialized = true;
  }
}

// static bool on_parameter_changed(const Parameter *old_p,
//                                  const Parameter *new_p,
//                                  void *)
// {
//   if (strcmp(new_p->name.data, "wheelbase") == 0 &&
//       new_p->value.type == RCLC_PARAMETER_DOUBLE) {
//     g_wheelbase = (float)new_p->value.double_value;
//     return true;
//   }
//   if (strcmp(new_p->name.data, "odom_period_ms") == 0 &&
//       new_p->value.type == RCLC_PARAMETER_INT) {
//     odometry_set_period_ms((unsigned int)new_p->value.integer_value);
//     return true;
//   }
//   if (strcmp(new_p->name.data, "wheel_radius") == 0 &&
//       new_p->value.type == RCLC_PARAMETER_DOUBLE) {
//     odometry_set_wheel_radius((float)new_p->value.double_value);
//     return true;
//   }
//   return false; // leave untouched if unknown/wrong type
// }

bool create_entities()
{
  allocator = rcl_get_default_allocator();
 
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
 
  // create node
  RCCHECK(rclc_node_init_default(&node, "car_controller", "", &support));
 
  // Motor RPM publisher disabled for stability
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &motor_rpm_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  //   "motor_rpms"));
 
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
 
  // Motor RPM timer disabled for stability
  // const unsigned int motor_rpm_timer_timeout = 100;
  // RCCHECK(rclc_timer_init_default(
  //   &motor_rpm_timer,
  //   &support,
  //   RCL_MS_TO_NS(motor_rpm_timer_timeout),
  //   motor_rpm_timer_callback));
 
  // num handles = total_of_subscribers + timers (publisher is not counted)
  // 2 subscriptions + 0 timers = 2 handles initially
  // odometry_init will add 1 more timer = 3 total handles
  unsigned int number_of_handles = 3;
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_handles, &allocator));
  // Motor RPM timer disabled
  // RCCHECK(rclc_executor_add_timer(&executor, &motor_rpm_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &geom_subscriber, &geom_msg,
    &vehicle_geometry_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &twist_subscriber, &twist_msg,
    &twist_callback, ON_NEW_DATA));
  // Ensure spin_some returns promptly to keep the loop responsive
  rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(50)); // Much more generous for stability

  // Parameter server on this node
  // rclc_parameter_server_init_default(&param_server, &node);

  // // Put the server into the executor (so set/get requests are handled)
  // rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);

  // // Declare parameters
  // rclc_add_parameter(&param_server, "wheelbase",      RCLC_PARAMETER_DOUBLE);
  // rclc_add_parameter(&param_server, "odom_period_ms", RCLC_PARAMETER_INT);
  // rclc_add_parameter(&param_server, "wheel_radius",   RCLC_PARAMETER_DOUBLE);

  // // Set initial values (mirrors your current defaults)
  // rclc_parameter_set_double(&param_server, "wheelbase",      param_wheelbase);
  // rclc_parameter_set_int   (&param_server, "odom_period_ms", g_odom_period_ms);
  // rclc_parameter_set_double(&param_server, "wheel_radius",   g_wheel_radius);

  // Push initial values into odometry
  odometry_set_period_ms((unsigned)g_odom_period_ms);
  odometry_set_wheel_radius((float)g_wheel_radius);

  if (!odometry_init(&node, &support, &executor)) {
    USBSerial.println("Failed to initialize odometry");
    return false;
  }

  return true;
}
 
void destroy_entities()
{
  if (car_initialized) {
    car.setSpeed(0.0f, g_wheelbase, g_track_width); // Start with zero speed
    car.setSteeringAngle(0.0f); // Start with zero steering angle
  }
 
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
 
  // Motor RPM publisher disabled
  // rcl_publisher_fini(&motor_rpm_publisher, &node);
  rcl_subscription_fini(&geom_subscriber, &node);
  rcl_subscription_fini(&twist_subscriber, &node);
  // Motor RPM timer disabled
  // rcl_timer_fini(&motor_rpm_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  // rclc_parameter_server_fini(&param_server, &node);
 
  odometry_fini(&node);
 
  // Motor RPM message disabled - no memory to free
  // if (motor_rpm_msg.data.data != NULL) {
  //   free(motor_rpm_msg.data.data);
  //   motor_rpm_msg.data.data = NULL;
  // }
}
 
void setup() {
  // Initialize LED pin for status indication
  pinMode(LED_PIN, OUTPUT);
  
  // LED test pattern - blink 3 times to confirm it's working
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  digitalWrite(LED_PIN, HIGH); // Keep LED on for testing
  
  // Initialize the car control system
  initializeCar();

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
  // motor_rpm_msg.data = 0;
 
  // Motor RPM message disabled for stability
  // motor_rpm_msg.data.size = 3;
  // motor_rpm_msg.data.capacity = 3;
  // motor_rpm_msg.data.data = (float*)malloc(3 * sizeof(float));
}
 
float angle = 40.00f;
uint32_t lastApplyMicros = micros();
 
void testMotorControl() {
  initializeCar();
 
  float steering_angle = car.steeringMotor.getSteeringAngle();
 
  // Set initial speed and angle
  car.setSpeed(200.0f, g_wheelbase, g_track_width);
 
  USBSerial.println("Time Elapsed");
  USBSerial.println(steering_angle);
  delay(50);
 
  const uint32_t now = micros();
  const bool timeElapsed = (now - lastApplyMicros) / 1000 >= 2000;
 
  if (!timeElapsed) {
    return;
  }
  lastApplyMicros = now;
  car.setSteeringAngle(angle);
  angle = -angle;
  USBSerial.println(angle);
}
 
 
void loop() {
  static uint32_t last_loop_time = 0;
  uint32_t current_time = millis();
  
  // Simple watchdog - if loop takes too long, something is blocking
  if (current_time - last_loop_time > 1000) {
    // System might be frozen, try to recover
    if (state == AGENT_CONNECTED) {
      state = AGENT_DISCONNECTED; // Force reconnection
    }
  }
  last_loop_time = current_time;
  
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
      EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  // Update LED status to show micro-ROS connection state
  updateLEDStatus();
  
  // Disable control loops completely to prevent blocking ROS
  // Control loops are too heavy and block the executor
  // static uint32_t last_control_update = 0;
  // uint32_t now = millis();
  // if (car_initialized && (now - last_control_update > 50)) {
  //   car.updateControlLoops();
  //   last_control_update = now;
  // }
  
  // No delay - maximum responsiveness for ROS
}