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
#include "odometry.h"
 
// Include car control logic
#include "car.h"
#include "debug.h"

#define CS_IMU      14
#define LED_PIN     37

// Global variable declarations (moved up to be accessible by all functions)
Car car(CS_RIGHT, CS_LEFT, CS_STEER);
bool car_initialized = false;
LSM6DSO IMU;

// Cached sensor data for non-blocking access
struct SensorData {
  float gyro_z = 0.0f;
  float right_rpm = 0.0f;
  float left_rpm = 0.0f;
  uint32_t last_update_ms = 0;
  SemaphoreHandle_t mutex;
} sensorData;
 
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

// Debug publisher - using simple approach
rcl_publisher_t debug_publisher;
std_msgs__msg__Float32MultiArray debug_msg;

// Static debug data array to avoid malloc/free
static float debug_data_array[35]; // Expanded for detailed debugging

// Additional ROS2 objects
rcl_subscription_t geom_subscriber;
geometry_msg__msg__VehicleGeometry geom_msg;
geometry_msgs__msg__Twist twist_msg;
 
// Vehicle geometry variables
float g_wheelbase = 0.185f;
float g_track_width = 0.15f;
 
// Desmos link: https://www.desmos.com/calculator/hzw38ukeor
float g_steering_scaling_factor = (1.0f/0.87f); // Scaling factor for encoder angle to wheel steering angle
float o_speed_scaling_factor = 4.0f; // Factor to scale the speed command
 
// Steering control constants
const float MAX_STEERING_ANGLE = 45.0f; // degrees
 
// Car initialization flags
TaskHandle_t updateTaskHandle = NULL;
TaskHandle_t microRosTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
 
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

// Thread-safe state management functions
agent_state_t getState() {
  agent_state_t current_state;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    current_state = state;
    xSemaphoreGive(stateMutex);
  } else {
    current_state = WAITING_AGENT; // Default to safe state on timeout
  }
  return current_state;
}

void setState(agent_state_t new_state) {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    state = new_state;
    xSemaphoreGive(stateMutex);
  }
}

// LED status variables
static uint32_t last_led_update = 0;
static uint32_t led_blink_count = 0;

// Debug counters
uint32_t twist_callback_count = 0;

// Performance monitoring variables
static uint32_t last_loop_start = 0;
static uint32_t max_loop_time = 0;
static uint32_t total_loop_time = 0;
static uint32_t loop_count = 0;
static uint32_t last_reset_reason = 0;

// Loop timing breakdown for debugging
static uint32_t max_ros_spin_time = 0;
static uint32_t max_sensor_time = 0;
static uint32_t max_control_time = 0;

// Detailed timing variables for debugging
static uint32_t max_steering_time = 0;
static uint32_t max_drive_time = 0;
static uint32_t max_odom_time = 0;
static uint32_t max_debug_publish_time = 0;
static uint32_t max_led_update_time = 0;

// State transition tracking
static uint32_t state_transition_count = 0;
static uint32_t last_state = 0;
static uint32_t connection_failures = 0;
static uint32_t executor_timeouts = 0;
 
typedef struct {
  TickType_t last_wake_time;
  TickType_t interval_ticks;
} periodic_task_t;

// Sensor reading task - runs in separate FreeRTOS task
void sensorTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // Update every 100ms
  uint32_t error_count = 0;
  
  while (true) {
    if (car_initialized) {
      // Add error handling for sensor reads
      float gyro_z = 0.0f;
      float right_rpm = 0.0f;
      float left_rpm = 0.0f;
      
      // Try to read sensors with timeout protection
      static uint32_t last_sensor_error = 0;
      uint32_t now = millis();
      
      // Only attempt sensor reads if no recent errors
      if (now - last_sensor_error > 1000) {
        // Read sensors with error handling
        gyro_z = IMU.readFloatGyroZ() * (M_PI / 180.0f);
        right_rpm = car.getRightMotorRPMAtomic();
        left_rpm = car.getLeftMotorRPMAtomic();
        
        // Update cached data with mutex protection (non-blocking)
        if (xSemaphoreTake(sensorData.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          sensorData.gyro_z = gyro_z;
          sensorData.right_rpm = right_rpm;
          sensorData.left_rpm = left_rpm;
          sensorData.last_update_ms = millis();
          xSemaphoreGive(sensorData.mutex);
          error_count = 0; // Reset error count on success
        } else {
          error_count++;
          if (error_count > 10) {
            last_sensor_error = now;
            error_count = 0;
          }
        }
      }
    }
    
    // Wait for next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// LED status update function
void updateLEDStatus() {
  uint32_t now = millis();
  if (now - last_led_update < 100) return; // Update every 100ms
  last_led_update = now;
  
  agent_state_t current_state = getState();
  switch (current_state) {
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
}

// Debug logging function - with proper timing and connection checks
void logDebug(const char* message) {
  // Only publish if connection is stable and we're not in the middle of setup
  if (getState() == AGENT_CONNECTED && car_initialized) {
    // Throttle debug messages to prevent overwhelming the system
    static uint32_t last_debug_time = 0;
    uint32_t now = millis();
    
     if (now - last_debug_time > 1000) { // Every 1 second
       // Use static array - no malloc/free needed
       debug_msg.data.data = debug_data_array;
       debug_msg.data.capacity = 35;
       debug_msg.data.size = 35;
      
      // Basic system metrics
      debug_msg.data.data[0] = (float)twist_callback_count;
      debug_msg.data.data[1] = (float)odom_state.publish_count;
      debug_msg.data.data[2] = (float)state;
      debug_msg.data.data[3] = (float)ESP.getFreeHeap(); // Free heap in bytes
      debug_msg.data.data[4] = (float)ESP.getHeapSize(); // Total heap size
      
      // Motor and sensor metrics
      debug_msg.data.data[5] = sensorData.right_rpm; // Right motor RPM
      debug_msg.data.data[6] = sensorData.left_rpm;  // Left motor RPM
      debug_msg.data.data[7] = sensorData.gyro_z;    // Gyro Z (rad/s)
      
      // Odometry position
      debug_msg.data.data[8] = odom_state.x;  // X position
      debug_msg.data.data[9] = odom_state.y; // Y position
      
      // ESP Performance metrics
      debug_msg.data.data[10] = (float)max_loop_time; // Max loop time (ms)
      debug_msg.data.data[11] = (float)(total_loop_time / max(1U, loop_count)); // Avg loop time (ms)
      debug_msg.data.data[12] = (float)ESP.getCpuFreqMHz(); // CPU frequency (MHz)
      debug_msg.data.data[13] = (float)ESP.getCycleCount() / 1000000.0f; // CPU cycles (millions)
      debug_msg.data.data[14] = (float)ESP.getFreePsram(); // Free PSRAM (bytes)
      debug_msg.data.data[15] = (float)ESP.getPsramSize(); // Total PSRAM (bytes)
      debug_msg.data.data[16] = 25.0f; // Temperature placeholder (C)
      debug_msg.data.data[17] = (float)last_reset_reason; // Reset reason code
      
      // Timing breakdown for debugging
      debug_msg.data.data[18] = (float)max_ros_spin_time; // Max ROS spin time (ms)
      debug_msg.data.data[19] = (float)max_sensor_time;   // Max sensor read time (ms)
      debug_msg.data.data[20] = (float)max_control_time;  // Max control time (ms)
      
      // Odometry publish status
      extern uint32_t odom_publish_success_count;
      extern uint32_t odom_publish_failure_count;
      debug_msg.data.data[21] = (float)odom_publish_success_count; // Successful publishes
      debug_msg.data.data[22] = (float)odom_publish_failure_count; // Failed publishes
      
       // Publisher initialization status (from odometry.cpp)
       extern rcl_ret_t publisher_init_result;
       debug_msg.data.data[23] = (float)publisher_init_result; // Publisher init result
       
       // Detailed timing metrics
       debug_msg.data.data[24] = (float)max_steering_time; // Max steering update time (ms)
       debug_msg.data.data[25] = (float)max_drive_time; // Max drive motor update time (ms)
       debug_msg.data.data[26] = (float)max_odom_time; // Max odometry time (ms)
       debug_msg.data.data[27] = (float)max_debug_publish_time; // Max debug publish time (ms)
       debug_msg.data.data[28] = (float)max_led_update_time; // Max LED update time (ms)
       
       // State transition tracking
       debug_msg.data.data[29] = (float)state_transition_count; // State transitions
       debug_msg.data.data[30] = (float)connection_failures; // Connection failures
       debug_msg.data.data[31] = (float)executor_timeouts; // Executor timeouts
       
       // Memory and task metrics
       debug_msg.data.data[32] = (float)uxTaskGetNumberOfTasks(); // Number of FreeRTOS tasks
       debug_msg.data.data[33] = (float)uxTaskGetStackHighWaterMark(NULL); // Main task stack high water mark
       debug_msg.data.data[34] = (float)(millis() % 100000); // System uptime (mod 100k for space)
       
       // Use non-blocking publish
       rcl_ret_t ret = rcl_publish(&debug_publisher, &debug_msg, NULL);
      if (ret != RCL_RET_OK) {
        // Store error for monitoring (avoid blocking in callback)
        static rcl_ret_t last_debug_error = RCL_RET_OK;
        last_debug_error = ret;
      }
      last_debug_time = now;
    }
  }
}
 
float getSteeringAngle(float omega, float vel) {
  if (omega == 0.0f || vel == 0.0f) {
    return 0.0f;
  }
  return atanf(g_wheelbase * omega / vel) * 180.0f / M_PI;
}
 
void twist_callback(const void * msgin) {
  twist_callback_count++; // Increment counter
  
  if (!car_initialized) {
    return; // Don't process if car not ready
  }
  
  const auto * twist = static_cast<const geometry_msgs__msg__Twist *>(msgin);
  
  // Value-based rate limiting: only process if values have changed significantly
  static float last_linear_x = 0.0f;
  static float last_angular_z = 0.0f;
  const float threshold = 0.01f; // 1cm/s or 0.01 rad/s threshold
  
  bool linear_changed = fabsf(twist->linear.x - last_linear_x) > threshold;
  bool angular_changed = fabsf(twist->angular.z - last_angular_z) > threshold;
  
  // Always process if either value changed significantly, or if both are zero (stop command)
  if (!linear_changed && !angular_changed && (twist->linear.x != 0.0f || twist->angular.z != 0.0f)) {
    return; // Skip processing if values haven't changed significantly
  }
  
  // Update stored values
  last_linear_x = twist->linear.x;
  last_angular_z = twist->angular.z;
 
  // Use bicycle model for proper steering angle calculation
  float steering_angle_deg = getSteeringAngle(twist->angular.z, twist->linear.x);
  float speed_rpm = twist->linear.x * o_speed_scaling_factor * 60.0f / (M_PI * 0.06f);
 
  // Update car control with safety limits
  // Limit steering angle to prevent extreme values
  if (steering_angle_deg > 30.0f) steering_angle_deg = 30.0f;
  if (steering_angle_deg < -30.0f) steering_angle_deg = -30.0f;
  
  // Limit speed to prevent extreme values
  if (speed_rpm > 1000.0f) speed_rpm = 1000.0f;
  if (speed_rpm < -1000.0f) speed_rpm = -1000.0f;
  
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

// Parameter callback function removed for simplicity

bool create_entities()
{
  allocator = rcl_get_default_allocator();
 
  // create init_options with timeout
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    return false;
  }
 
  // create node with timeout
  ret = rclc_node_init_default(&node, "car_controller", "", &support);
  if (ret != RCL_RET_OK) {
    rclc_support_fini(&support);
    return false;
  }
 
  // Motor RPM publisher disabled for stability
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &motor_rpm_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  //   "motor_rpms"));

  // create debug publisher with error handling
  ret = rclc_publisher_init_best_effort(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "debug_log");
  if (ret != RCL_RET_OK) {
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
    return false;
  }
  
  // create subscriber for vehicle geometry with error handling
  ret = rclc_subscription_init_best_effort(
    &geom_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msg, msg, VehicleGeometry),
    "vehicle_geometry");
  if (ret != RCL_RET_OK) {
    (void)rcl_publisher_fini(&debug_publisher, &node);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
    return false;
  }
  
  // create subscriber for twist messages (cmd_vel) with error handling
  ret = rclc_subscription_init_best_effort(
    &twist_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");
  if (ret != RCL_RET_OK) {
    (void)rcl_subscription_fini(&geom_subscriber, &node);
    (void)rcl_publisher_fini(&debug_publisher, &node);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
    return false;
  }
 
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
  unsigned int number_of_handles = 3;  // With odometry
  executor = rclc_executor_get_zero_initialized_executor();
  ret = rclc_executor_init(&executor, &support.context, number_of_handles, &allocator);
  if (ret != RCL_RET_OK) {
    (void)rcl_subscription_fini(&twist_subscriber, &node);
    (void)rcl_subscription_fini(&geom_subscriber, &node);
    (void)rcl_publisher_fini(&debug_publisher, &node);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
    return false;
  }
  
  // Motor RPM timer disabled
  // RCCHECK(rclc_executor_add_timer(&executor, &motor_rpm_timer));
  ret = rclc_executor_add_subscription(&executor, &geom_subscriber, &geom_msg,
    &vehicle_geometry_callback, ON_NEW_DATA);
  if (ret != RCL_RET_OK) {
    (void)rclc_executor_fini(&executor);
    (void)rcl_subscription_fini(&twist_subscriber, &node);
    (void)rcl_subscription_fini(&geom_subscriber, &node);
    (void)rcl_publisher_fini(&debug_publisher, &node);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
    return false;
  }
  
  ret = rclc_executor_add_subscription(
    &executor, &twist_subscriber, &twist_msg,
    &twist_callback, ON_NEW_DATA);
  if (ret != RCL_RET_OK) {
    (void)rclc_executor_fini(&executor);
    (void)rcl_subscription_fini(&twist_subscriber, &node);
    (void)rcl_subscription_fini(&geom_subscriber, &node);
    (void)rcl_publisher_fini(&debug_publisher, &node);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
    return false;
  }
   // Ensure spin_some returns promptly to keep the loop responsive
   rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1));

  // Parameter server functionality removed for simplicity

  // Initialize odometry with conservative settings
  odometry_set_period_ms(100); // 10Hz for stability
  odometry_set_wheel_radius(0.03f); // 3cm radius

  if (!odometry_init(&node, &support, &executor)) {
    (void)rclc_executor_fini(&executor);
    (void)rcl_subscription_fini(&twist_subscriber, &node);
    (void)rcl_subscription_fini(&geom_subscriber, &node);
    (void)rcl_publisher_fini(&debug_publisher, &node);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
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
  rmw_ret_t rmw_ret = rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  if (rmw_ret != RMW_RET_OK) {
    // Handle rmw error if needed
  }

  // Motor RPM publisher disabled
  // rcl_publisher_fini(&motor_rpm_publisher, &node);
  rcl_ret_t ret1 = rcl_subscription_fini(&geom_subscriber, &node);
  rcl_ret_t ret2 = rcl_subscription_fini(&twist_subscriber, &node);
  // Motor RPM timer disabled
  // rcl_timer_fini(&motor_rpm_timer);
  rcl_ret_t ret3 = rcl_publisher_fini(&debug_publisher, &node);
  
  if (ret1 != RCL_RET_OK || ret2 != RCL_RET_OK || ret3 != RCL_RET_OK) {
    // Handle cleanup errors if needed
  }

  // Clean up odometry
  odometry_fini(&node);
  
  rcl_ret_t ret4 = rclc_executor_fini(&executor);
  rcl_ret_t ret5 = rcl_node_fini(&node);
  rcl_ret_t ret6 = rclc_support_fini(&support);
  
  // Suppress unused variable warnings for cleanup
  (void)ret4;
  (void)ret5;
  (void)ret6;
  // rclc_parameter_server_fini(&param_server, &node);
 
  // Motor RPM message disabled - no memory to free
  // if (motor_rpm_msg.data.data != NULL) {
  //   free(motor_rpm_msg.data.data);
  //   motor_rpm_msg.data.data = NULL;
  // }
}
 
void setup() {
  // Capture reset reason for debugging
  last_reset_reason = 0; // Reset reason (simplified)
  
  // Initialize LED pin for status indication
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize the car control system
  initializeCar();

  // Initialize USB CDC with timeout protection
  USB.begin();
  USBSerial.begin(921600);
  
  // Wait for USB to be ready with timeout (max 5 seconds)
  uint32_t usb_timeout = millis();
  while (!USBSerial && (millis() - usb_timeout < 5000)) {
    delay(10);
  }
  
  // Only proceed if USB is ready, otherwise continue with limited functionality
  if (!USBSerial) {
    // USB failed to initialize, but continue with system
    // micro-ROS will handle connection retries
  }

    
  // LED test pattern - blink 3 times to confirm it's working
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
 
  // Configure Micro-ROS library to use USB CDC serial
  set_microros_serial_transports(USBSerial);
 
  state = WAITING_AGENT;  
  // motor_rpm_msg.data = 0;
  
  // Initialize debug message
  debug_msg.data.size = 0;
  debug_msg.data.capacity = 0;
  debug_msg.data.data = NULL;
  
  // Initialize sensor data mutex
  sensorData.mutex = xSemaphoreCreateMutex();
  
  // Initialize state mutex for thread-safe state management
  stateMutex = xSemaphoreCreateMutex();
  
  // Create sensor reading task with lower priority and smaller stack
  BaseType_t task_result = xTaskCreatePinnedToCore(
    sensorTask,           // Task function
    "SensorTask",         // Task name
    1024,                 // Reduced stack size
    NULL,                 // Parameters
    1,                    // Lower priority (same as main loop)
    &sensorTaskHandle,    // Task handle
    1                     // Core 1 (leave core 0 for main loop)
  );
  
  if (task_result != pdPASS) {
    // Task creation failed, but continue with system
    // Sensor reading will be handled in main loop
  }
}
 
float angle = 40.00f;
uint32_t lastApplyMicros = micros();
 
void testMotorControl() {
  initializeCar();

  float steering_angle = car.steeringMotor.getSteeringAngle();

  // Set initial speed and angle
  car.setSpeed(200.0f, g_wheelbase, g_track_width);

  // Read and print IMU data
  if (car_initialized) {
    float accel_x = IMU.readFloatAccelX();
    float accel_y = IMU.readFloatAccelY();
    float accel_z = IMU.readFloatAccelZ();
    float gyro_x = IMU.readFloatGyroX();
    float gyro_y = IMU.readFloatGyroY();
    float gyro_z = IMU.readFloatGyroZ();
    float temperature = IMU.readTempF();
    
    USBSerial.println("=== IMU Data ===");
    USBSerial.print("Accel X: "); USBSerial.print(accel_x); USBSerial.println(" g");
    USBSerial.print("Accel Y: "); USBSerial.print(accel_y); USBSerial.println(" g");
    USBSerial.print("Accel Z: "); USBSerial.print(accel_z); USBSerial.println(" g");
    USBSerial.print("Gyro X: "); USBSerial.print(gyro_x); USBSerial.println(" dps");
    USBSerial.print("Gyro Y: "); USBSerial.print(gyro_y); USBSerial.println(" dps");
    USBSerial.print("Gyro Z: "); USBSerial.print(gyro_z); USBSerial.println(" dps");
    USBSerial.print("Temperature: "); USBSerial.print(temperature); USBSerial.println(" F");
    USBSerial.println("================");
  }

  USBSerial.println("Time Elapsed");
  USBSerial.println(steering_angle);

  car.setSteeringAngle(angle);
  angle = -angle;
  USBSerial.println(angle);
}
 
 
void loop() {
  static uint32_t last_loop_time = 0;
  uint32_t current_time = millis();
  
  // Performance monitoring
  if (last_loop_start > 0) {
    uint32_t loop_time = current_time - last_loop_start;
    if (loop_time > max_loop_time) {
      max_loop_time = loop_time;
    }
    total_loop_time += loop_time;
    loop_count++;
  }
  last_loop_start = current_time;
  
   // Simple watchdog - if loop takes too long, something is blocking
   if (current_time - last_loop_time > 1000) {
     // System might be frozen, try to recover
     if (getState() == AGENT_CONNECTED) {
       setState(AGENT_DISCONNECTED); // Force reconnection
     }
   }
  last_loop_time = current_time;
  
  // Time ROS executor operations with watchdog
  uint32_t ros_start = millis();
  static uint32_t ros_spin_timeout_count = 0;
  
   // Track state transitions
   agent_state_t current_state = getState();
   if (current_state != last_state) {
     state_transition_count++;
     last_state = current_state;
   }
   
   // Declare variables outside switch to avoid jump to case label errors
   agent_state_t new_state;
   uint32_t ros_start_inner;
   uint32_t ros_time_inner;
   
   switch (current_state) {
     case WAITING_AGENT:
       EXECUTE_EVERY_N_MS(500, setState((RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT););
       break;
     case AGENT_AVAILABLE:
       new_state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
       setState(new_state);
       if (new_state == WAITING_AGENT) {
         connection_failures++;
         destroy_entities();
       }
       break;
     case AGENT_CONNECTED:
       EXECUTE_EVERY_N_MS(500, setState((RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED););
       if (getState() == AGENT_CONNECTED) {
         // Use very aggressive timeout to prevent blocking
         ros_start_inner = millis();
         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); // 1ms timeout
         ros_time_inner = millis() - ros_start_inner;
         
         // Watchdog: if ROS takes too long, force disconnect
         if (ros_time_inner > 5) {
           executor_timeouts++;
           if (executor_timeouts > 3) {
             setState(AGENT_DISCONNECTED); // Force reconnection
             executor_timeouts = 0;
           }
         } else {
           executor_timeouts = 0; // Reset on good performance
         }
       }
       break;
     case AGENT_DISCONNECTED:
       destroy_entities();
       setState(WAITING_AGENT);
       break;
     default:
       break;
   }
  
  uint32_t ros_time = millis() - ros_start;
  if (ros_time > max_ros_spin_time) {
    max_ros_spin_time = ros_time;
  }

  // Update LED status to show micro-ROS connection state
  updateLEDStatus();
  
  // Debug logging - only in main loop, properly throttled
  static uint32_t last_debug_output = 0;
  uint32_t now = millis();
  if (now - last_debug_output > 10000) { // Every 10 seconds
    logDebug("status");
    last_debug_output = now;
  }
  
  // Time control operations
  uint32_t control_start = millis();
  
   if (car_initialized) {    
     // Update steering more frequently for responsiveness (every 10ms)
     static uint32_t last_steering_update = 0;
     if (now - last_steering_update >= 10) { // 100Hz steering update
       uint32_t steer_start = millis();
       car.steeringMotor.updatePosition();
       uint32_t steer_time = millis() - steer_start;
       if (steer_time > max_steering_time) {
         max_steering_time = steer_time;
       }
       last_steering_update = now;
     }
     
     // Update drive motors less frequently (every 20ms) for efficiency
     static uint32_t last_control_update = 0;
     if (now - last_control_update >= 20) { // 50Hz control loop
       uint32_t drive_start = millis();
       car.updateControlLoops();
       uint32_t drive_time = millis() - drive_start;
       if (drive_time > max_drive_time) {
         max_drive_time = drive_time;
       }
       last_control_update = now;
     }
   }
  
  uint32_t control_time = millis() - control_start;
  if (control_time > max_control_time) {
    max_control_time = control_time;
  }
}