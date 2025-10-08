#include <Arduino.h>
#include "USB.h"
#include "USBCDC.h"
#include <micro_ros_platformio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Include our new modular classes
#include "ros_interface.h"
#include "sensor_manager.h"
#include "car.h"
#include "debug.h"

// Pin definitions
#define CS_IMU      14
#define LED_PIN     37

// Global objects
Car car(CS_RIGHT, CS_LEFT, CS_STEER);
bool car_initialized = false;
USBCDC USBSerial;
 
// Modular components
ROSInterface ros_interface;
SensorManager sensor_manager;

// LED status variables
static uint32_t last_led_update = 0;
static uint32_t led_blink_count = 0;

// Performance monitoring
static uint32_t max_loop_time = 0;
static uint32_t total_loop_time = 0;
static uint32_t loop_count = 0;

void updateLEDStatus() {
  uint32_t now = millis();
    if (now - last_led_update < 100) return;
  last_led_update = now;
  
    ROSInterface::ConnectionState state = ros_interface.getState();
    switch (state) {
        case ROSInterface::WAITING_AGENT:
      digitalWrite(LED_PIN, (led_blink_count / 10) % 2);
      break;
        case ROSInterface::AGENT_AVAILABLE:
      digitalWrite(LED_PIN, (led_blink_count / 2) % 2);
      break;
        case ROSInterface::AGENT_CONNECTED:
      digitalWrite(LED_PIN, HIGH);
      break;
        case ROSInterface::AGENT_DISCONNECTED:
      digitalWrite(LED_PIN, (led_blink_count / 5) % 4 < 2);
      break;
  }
  led_blink_count++;
}
 
void initializeCar() {
  if (!car_initialized) {    
    // Initialize SPI for motor drivers
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
    delay(100);
 
    // Initialize car control
    car.begin();
    delay(100);
 
        car.setSpeed(0.0f, ros_interface.getWheelbase(), ros_interface.getTrackWidth());
        car.setSteeringAngle(0.0f);
   
    car_initialized = true;
  }
}
 
void setup() {
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize USB CDC for micro-ROS
  USB.begin();
  USBSerial.begin(921600);
  
    // Wait for USB with timeout
  uint32_t usb_timeout = millis();
  while (!USBSerial && (millis() - usb_timeout < 5000)) {
    delay(10);
  }
  
    // LED test pattern
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
 
  // Configure micro-ROS transport
  set_microros_serial_transports(USBSerial);
 
  // Initialize car
  initializeCar();
  sensor_manager.initialize();
  ros_interface.initialize();
}
 
void loop() {
  static uint32_t last_loop_start = 0;
  static uint32_t last_watchdog_reset = 0;
  uint32_t current_time = millis();
  
  // Watchdog safety mechanism - reset if loop takes too long
  if (last_loop_start > 0) {
    uint32_t loop_time = current_time - last_loop_start;
    if (loop_time > 1000) {  // If loop takes more than 1 second, something is wrong
      // Force stop motors and reset
      car.setSpeed(0.0f, ros_interface.getWheelbase(), ros_interface.getTrackWidth());
      car.setSteeringAngle(0.0f);
      last_watchdog_reset = current_time;
    }
  }
  
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
  
  // System protection against message flooding
  static uint32_t last_ros_update = 0;
  static uint32_t last_sensor_update = 0;
  
  // TEMPORARILY DISABLED - Test if ROS interface is causing freeze
  // if (current_time - last_ros_update >= 10) {  // 100Hz max
  //   ros_interface.update();
  //   last_ros_update = current_time;
  // }
  
  // TEMPORARILY DISABLED - Test if sensor manager is causing freeze
  // if (current_time - last_sensor_update >= 20) {  // 50Hz max
  //   sensor_manager.update();
  //   last_sensor_update = current_time;
  // }
    
    // TEMPORARILY DISABLED - Test if car control is causing freeze
    // if (ros_interface.isConnected() && car_initialized) {
    //     // Update steering (10Hz) - much slower to prevent issues
    //     static uint32_t last_steering_update = 0;
    //     if (current_time - last_steering_update >= 100) {
    //         car.steeringMotor.updatePosition();
    //         last_steering_update = current_time;
    //     }
    //     
    //     // Update drive motors (10Hz) - much slower to prevent issues
    //     static uint32_t last_control_update = 0;
    //     if (current_time - last_control_update >= 100) {
    //         car.updateControlLoops();
    //         last_control_update = current_time;
    //     }
    // }
    
    // TEMPORARILY DISABLED - Test if publishing is causing freeze
    // if (ros_interface.isConnected()) {
    //     static uint32_t last_imu_publish = 0;
    //     static uint32_t last_motor_publish = 0;
    //     
    //     SensorData sensor_data = sensor_manager.getLatestData();
    //     if (sensor_manager.isDataValid()) {
    //         // Publish IMU data at 10Hz
    //         if (current_time - last_imu_publish >= 100) {
    //             ros_interface.publishIMUData(sensor_data);
    //             last_imu_publish = current_time;
    //         }
    //         
    //         // Publish motor data at 10Hz
    //         if (current_time - last_motor_publish >= 100) {
    //             ros_interface.publishMotorData(car);
    //             last_motor_publish = current_time;
    //         }
    //     }
    //     
    //     // Publish debug data every 10 seconds
    //     static uint32_t last_debug_publish = 0;
    //     if (current_time - last_debug_publish >= 500) {
    //         float debug_data[20] = {
    //             (float)ros_interface.getState(),
    //             (float)ESP.getFreeHeap(),
    //             (float)ESP.getHeapSize(),
    //             (float)max_loop_time,
    //             (float)(total_loop_time / max(1U, loop_count)),
    //             sensor_data.gyro_z,
    //             sensor_data.right_rpm,
    //             sensor_data.left_rpm,
    //             sensor_data.temperature,
    //             (float)ESP.getCpuFreqMHz(),
    //             (float)ESP.getFreePsram(),
    //             (float)ESP.getPsramSize(),
    //             (float)uxTaskGetNumberOfTasks(),
    //             (float)uxTaskGetStackHighWaterMark(NULL),
    //             (float)(millis() % 100000),
    //             0.0f, 0.0f, 0.0f, 0.0f, 0.0f // Reserved for future use
    //         };
    //         
    //         ros_interface.publishDebugData(debug_data, 20);
    //         last_debug_publish = current_time;
    //     }
    // }
    
    // Update LED status
    updateLEDStatus();
    
    // Very conservative delay to prevent overwhelming the system
    delay(1);  // 50ms delay to limit main loop to ~20Hz max
}