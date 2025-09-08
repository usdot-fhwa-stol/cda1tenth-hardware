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

// Declare USBSerial object
USBCDC USBSerial;

void setup() {
  // Initialize the entire system using the system initializer
  if (!initializeSystem()) {
    // If initialization fails, we can't do much without USB
    // The system initializer will have logged error messages
    while (true) {
      delay(1000); // Wait and retry
      if (initializeSystem()) {
        break;
      }
    }
  }
  
  // Log successful initialization via ROS logger
  ROS_LOG_INFO("MAIN", "System initialization completed successfully");
}

// Legacy testMotorControl function removed


void loop() {
  // Main loop now just monitors system health and provides status updates
  static uint32_t lastStatusUpdate = 0;
  static uint32_t lastSerialDebug = 0;
  uint32_t currentTime = millis();
  
  // Print serial debug every 5 seconds (for standalone debugging)
  if (currentTime - lastSerialDebug >= 5000) {
    USBSerial.println("=== SYSTEM DEBUG ===");
    USBSerial.printf("System Healthy: %s\n", isSystemHealthy() ? "YES" : "NO");
    USBSerial.printf("System Initialized: %s\n", isSystemInitialized() ? "YES" : "NO");
    
    ROSCommTask* rosTask = getROSCommTask();
    if (rosTask != nullptr) {
      USBSerial.printf("ROS Task Running: %s\n", rosTask->isRunning() ? "YES" : "NO");
      USBSerial.printf("ROS Agent Connected: %s\n", rosTask->isAgentConnected() ? "YES" : "NO");
    } else {
      USBSerial.println("ROS Task: NULL");
    }
    USBSerial.println("===================");
    lastSerialDebug = currentTime;
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