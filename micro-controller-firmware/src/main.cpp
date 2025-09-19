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
#include <math.h>

// Include car control logic
#include "car.h"
#include "inter_core_comm.h"
#include "ros_comm_task.h"
#include "system_init.h"
#include "ros_logger.h"
#include "web_debug_server.h"

// Declare USBSerial object
USBCDC USBSerial;

void setup() {
  // Show setup started with LED
  pinMode(2, OUTPUT); // Built-in LED
  digitalWrite(2, HIGH); // Turn on built-in LED
  
  // Initialize the entire system using the system initializer
  if (!initializeSystem()) {
    // If initialization fails, just continue with limited functionality
    // Don't retry to prevent boot loops
    digitalWrite(2, LOW); // Turn off LED to show failure
    delay(5000); // Wait 5 seconds before continuing
  } else {
    digitalWrite(2, LOW); // Turn off LED to show success
  }
  
  // Log successful initialization via ROS logger (if available)
  ROS_LOG_INFO("MAIN", "System initialization completed successfully");
  
  // Log to web debug server (if available)
  webDebugServer.logInfo("MAIN", "System initialization completed successfully");
  webDebugServer.updateSystemStatus("System initialized and running");
}

// Legacy testMotorControl function removed


void loop() {
  // Main loop now just monitors system health and provides status updates
  static uint32_t lastStatusUpdate = 0;
  static uint32_t lastWebUpdate = 0;
  static uint32_t lastBlink = 0;
  uint32_t currentTime = millis();
  
  // Blink built-in LED every 2 seconds to show main loop is running
  if (currentTime - lastBlink >= 2000) {
    digitalWrite(2, !digitalRead(2)); // Toggle LED
    lastBlink = currentTime;
  }
  
  // Update web debug server every 2 seconds
  if (currentTime - lastWebUpdate >= 2000) {
    // Add heartbeat message to verify web debug server is working
    static uint32_t heartbeatCounter = 0;
    heartbeatCounter++;
    
    // Add heartbeat message (with safety check)
    if (heartbeatCounter % 10 == 0) { // Only log every 10th heartbeat to reduce load
      webDebugServer.logInfo("MAIN", "Main loop heartbeat: " + String(heartbeatCounter));
    }
    
    // Update performance metrics for web interface
    ROSCommTask* rosTask = getROSCommTask();
    MultiCoreCar* motorCar = getMultiCoreCar();
    
    String perfMetrics = "Uptime: " + String(millis() / 1000) + "s";
    if (rosTask != nullptr && rosTask->isRunning()) {
      ROSCommPerformanceMetrics rosMetrics = rosTask->getPerformanceMetrics();
      perfMetrics += " | ROS Executions: " + String(rosMetrics.execution_count);
      perfMetrics += " | ROS Failures: " + String(rosMetrics.ros_message_send_failures);
    }
    if (motorCar != nullptr) {
      MotorControlPerformanceMetrics motorMetrics = motorCar->getPerformanceMetrics();
      perfMetrics += " | Motor Executions: " + String(motorMetrics.execution_count);
      perfMetrics += " | Motor Errors: " + String(motorMetrics.spi_errors);
    }
    
    webDebugServer.updatePerformanceMetrics(perfMetrics);
    lastWebUpdate = currentTime;
  }
  
  // Print detailed status every 10 seconds
  if (currentTime - lastStatusUpdate >= 10000) {
    printSystemStatus();
    
    // Print additional performance metrics
    ROSCommTask* rosTask = getROSCommTask();
    MultiCoreCar* motorCar = getMultiCoreCar();
    
    if (rosTask != nullptr && rosTask->isRunning()) {
      ROSCommPerformanceMetrics rosMetrics = rosTask->getPerformanceMetrics();
      ROS_LOG_INFOF("PERF", "ROS Performance - Executions: %u, Avg Time: %u us, Send Failures: %u, Connection Drops: %u",
        rosMetrics.execution_count, rosMetrics.avg_execution_time_us, 
        rosMetrics.ros_message_send_failures, rosMetrics.ros_connection_drops);
    }
    
    if (motorCar != nullptr) {
      MotorControlPerformanceMetrics motorMetrics = motorCar->getPerformanceMetrics();
      ROS_LOG_INFOF("PERF", "Motor Performance - Executions: %u, Avg Time: %u us, Missed Deadlines: %u, SPI Errors: %u",
        motorMetrics.execution_count, motorMetrics.avg_execution_time_us,
        motorMetrics.missed_deadlines, motorMetrics.spi_errors);
    }
    
    lastStatusUpdate = currentTime;
  }
  
  // Check system health and attempt recovery if needed
  if (!isSystemHealthy()) {
    static uint32_t lastHealthCheck = 0;
    if (currentTime - lastHealthCheck >= 5000) {
      ROS_LOG_WARN("HEALTH", "System health check failed - attempting recovery");
      // Could implement recovery logic here
      lastHealthCheck = currentTime;
    }
  }
  
  // Small delay to prevent overwhelming the system
  delay(100);
}